// Lifted from MuJoCo (CDR-3, pin mjVERSION_HEADER 3010000, Apache-2.0, (c)
// DeepMind Technologies Limited -- see NOTICE). The numeric kernels below are
// lifted verbatim from the vendored user-layer math pool and registered in
// snapshots/lifted_code.json (ids: resolve_orientation, full_inertia; sources
// user_objects.cc ResolveOrientation and user_util.cc mjuu_fullInertia + its
// mjuu_* dependencies). This module is MuJoCo-free (no mujoco.h) so the reader
// (protospec_io) can canonicalize orientation/inertia at parse end without
// linking MuJoCo (plan.md Section 3 core/resolve). The mjuu_* helpers are a
// self-contained copy of the same functions in attic/compile/lifted/mjuu_util.cc;
// they carry no state and are drift-gated against the upstream originals.
#include "resolve.h"

#include <cmath>

namespace ps::core {
namespace {

// Constants (verbatim from mjuu_util.h). mjPI matches MuJoCo's mjPI.
constexpr double mjEPS = 1E-14;
constexpr double mjPI = 3.14159265358979323846;

// --- mjuu_* vector/quaternion/matrix helpers (verbatim, user_util.cc) ------ //
bool mjuu_defined(double num) { return !std::isnan(num); }

void mjuu_setvec(double* dest, double x, double y, double z, double w) {
  dest[0] = x; dest[1] = y; dest[2] = z; dest[3] = w;
}

template <typename T1, typename T2>
void mjuu_copyvec(T1* dest, const T2* src, int n) {
  for (int i = 0; i < n; i++) dest[i] = (T1)src[i];
}

double mjuu_dot3(const double* a, const double* b) {
  return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

// normalize vector to unit length, return previous length
double mjuu_normvec(double* vec, const int n) {
  double nrm = 0;
  for (int i = 0; i < n; i++) nrm += vec[i]*vec[i];
  if (nrm < mjEPS) return 0;
  nrm = std::sqrt(nrm);
  if (std::abs(nrm - 1) > mjEPS) {
    for (int i = 0; i < n; i++) vec[i] /= nrm;
  }
  return nrm;
}

void mjuu_crossvec(double* a, const double* b, const double* c) {
  a[0] = b[1]*c[2] - b[2]*c[1];
  a[1] = b[2]*c[0] - b[0]*c[2];
  a[2] = b[0]*c[1] - b[1]*c[0];
}

// convert unit quaternion to 3-by-3 rotation matrix
void mjuu_quat2mat(double* res, const double* quat) {
  if (quat[0] == 1 && quat[1] == 0 && quat[2] == 0 && quat[3] == 0) {
    res[0] = 1; res[1] = 0; res[2] = 0;
    res[3] = 0; res[4] = 1; res[5] = 0;
    res[6] = 0; res[7] = 0; res[8] = 1;
    return;
  }
  double q00 = quat[0]*quat[0], q01 = quat[0]*quat[1], q02 = quat[0]*quat[2];
  double q03 = quat[0]*quat[3], q11 = quat[1]*quat[1], q12 = quat[1]*quat[2];
  double q13 = quat[1]*quat[3], q22 = quat[2]*quat[2], q23 = quat[2]*quat[3];
  double q33 = quat[3]*quat[3];
  res[0] = q00 + q11 - q22 - q33;
  res[4] = q00 - q11 + q22 - q33;
  res[8] = q00 - q11 - q22 + q33;
  res[1] = 2*(q12 - q03); res[2] = 2*(q13 + q02);
  res[3] = 2*(q12 + q03); res[5] = 2*(q23 - q01);
  res[6] = 2*(q13 - q02); res[7] = 2*(q23 + q01);
}

// multiply two unit quaternions
void mjuu_mulquat(double* res, const double* qa, const double* qb) {
  double tmp[4];
  tmp[0] = qa[0]*qb[0] - qa[1]*qb[1] - qa[2]*qb[2] - qa[3]*qb[3];
  tmp[1] = qa[0]*qb[1] + qa[1]*qb[0] + qa[2]*qb[3] - qa[3]*qb[2];
  tmp[2] = qa[0]*qb[2] - qa[1]*qb[3] + qa[2]*qb[0] + qa[3]*qb[1];
  tmp[3] = qa[0]*qb[3] + qa[1]*qb[2] - qa[2]*qb[1] + qa[3]*qb[0];
  mjuu_normvec(tmp, 4);
  mjuu_copyvec(res, tmp, 4);
}

// multiply two matrices, all 3-by-3: res = A * B
void mjuu_mulmat(double* res, const double* A, const double* B) {
  double tmp[9];
  tmp[0] = A[0]*B[0] + A[1]*B[3] + A[2]*B[6];
  tmp[1] = A[0]*B[1] + A[1]*B[4] + A[2]*B[7];
  tmp[2] = A[0]*B[2] + A[1]*B[5] + A[2]*B[8];
  tmp[3] = A[3]*B[0] + A[4]*B[3] + A[5]*B[6];
  tmp[4] = A[3]*B[1] + A[4]*B[4] + A[5]*B[7];
  tmp[5] = A[3]*B[2] + A[4]*B[5] + A[5]*B[8];
  tmp[6] = A[6]*B[0] + A[7]*B[3] + A[8]*B[6];
  tmp[7] = A[6]*B[1] + A[7]*B[4] + A[8]*B[7];
  tmp[8] = A[6]*B[2] + A[7]*B[5] + A[8]*B[8];
  mjuu_copyvec(res, tmp, 9);
}

// transpose 3-by-3 matrix
void mjuu_transposemat(double* res, const double* mat) {
  double tmp[9] = {mat[0], mat[3], mat[6],
                   mat[1], mat[4], mat[7],
                   mat[2], mat[5], mat[8]};
  mjuu_copyvec(res, tmp, 9);
}

// compute quaternion as minimal rotation from [0;0;1] to vec
void mjuu_z2quat(double* quat, const double* vec) {
  double z[3] = {0, 0, 1};
  (void)z;
  mjuu_crossvec(quat+1, z, vec);
  double s = mjuu_normvec(quat+1, 3);
  if (s < 1E-10) {
    quat[1] = 1;
    quat[2] = quat[3] = 0;
  }
  double ang = std::atan2(s, vec[2]);
  quat[0] = std::cos(ang/2);
  quat[1] *= std::sin(ang/2);
  quat[2] *= std::sin(ang/2);
  quat[3] *= std::sin(ang/2);
}

// compute quaternion given frame (axes are in matrix columns)
void mjuu_frame2quat(double* quat, const double* x, const double* y,
                     const double* z) {
  const double* mat[3] = {x, y, z};  // mat[c][r] indexing
  if (mat[0][0]+mat[1][1]+mat[2][2] > 0) {
    quat[0] = 0.5 * std::sqrt(1 + mat[0][0] + mat[1][1] + mat[2][2]);
    quat[1] = 0.25 * (mat[1][2] - mat[2][1]) / quat[0];
    quat[2] = 0.25 * (mat[2][0] - mat[0][2]) / quat[0];
    quat[3] = 0.25 * (mat[0][1] - mat[1][0]) / quat[0];
  } else if (mat[0][0] > mat[1][1] && mat[0][0] > mat[2][2]) {
    quat[1] = 0.5 * std::sqrt(1 + mat[0][0] - mat[1][1] - mat[2][2]);
    quat[0] = 0.25 * (mat[1][2] - mat[2][1]) / quat[1];
    quat[2] = 0.25 * (mat[1][0] + mat[0][1]) / quat[1];
    quat[3] = 0.25 * (mat[2][0] + mat[0][2]) / quat[1];
  } else if (mat[1][1] > mat[2][2]) {
    quat[2] = 0.5 * std::sqrt(1 - mat[0][0] + mat[1][1] - mat[2][2]);
    quat[0] = 0.25 * (mat[2][0] - mat[0][2]) / quat[2];
    quat[1] = 0.25 * (mat[1][0] + mat[0][1]) / quat[2];
    quat[3] = 0.25 * (mat[2][1] + mat[1][2]) / quat[2];
  } else {
    quat[3] = 0.5 * std::sqrt(1 - mat[0][0] - mat[1][1] + mat[2][2]);
    quat[0] = 0.25 * (mat[0][1] - mat[1][0]) / quat[3];
    quat[1] = 0.25 * (mat[2][0] + mat[0][2]) / quat[3];
    quat[2] = 0.25 * (mat[2][1] + mat[1][2]) / quat[3];
  }
  mjuu_normvec(quat, 4);
}

// eigenvalue decomposition of symmetric 3x3 matrix (Jacobi)
constexpr double kEigEPS = 1E-12;
int mjuu_eig3(double eigval[3], double eigvec[9], double quat[4],
              const double mat[9]) {
  double D[9], tmp[9], tmp2[9];
  double tau, t, c;
  int iter, rk, ck, rotk;

  quat[0] = 1;
  quat[1] = quat[2] = quat[3] = 0;

  for (iter = 0; iter < 500; iter++) {
    // make quaternion matrix eigvec, compute D = eigvec'*mat*eigvec
    mjuu_quat2mat(eigvec, quat);
    mjuu_transposemat(tmp2, eigvec);
    mjuu_mulmat(tmp, tmp2, mat);
    mjuu_mulmat(D, tmp, eigvec);

    eigval[0] = D[0];
    eigval[1] = D[4];
    eigval[2] = D[8];

    if (std::abs(D[1]) > std::abs(D[2]) && std::abs(D[1]) > std::abs(D[5])) {
      rk = 0; ck = 1; rotk = 2;
    } else if (std::abs(D[2]) > std::abs(D[5])) {
      rk = 0; ck = 2; rotk = 1;
    } else {
      rk = 1; ck = 2; rotk = 0;
    }

    if (std::abs(D[3*rk+ck]) < kEigEPS) break;

    tau = (D[4*ck]-D[4*rk])/(2*D[3*rk+ck]);
    if (tau >= 0) {
      t = 1.0/(tau + std::sqrt(1 + tau*tau));
    } else {
      t = -1.0/(-tau + std::sqrt(1 + tau*tau));
    }
    c = 1.0/std::sqrt(1 + t*t);

    if (c > 1.0-kEigEPS) break;

    tmp[1] = tmp[2] = tmp[3] = 0;
    tmp[rotk+1] = (tau >= 0 ? -std::sqrt(0.5-0.5*c) : std::sqrt(0.5-0.5*c));
    if (rotk == 1) tmp[rotk+1] = -tmp[rotk+1];
    tmp[0] = std::sqrt(1.0 - tmp[rotk+1]*tmp[rotk+1]);
    mjuu_normvec(tmp, 4);

    mjuu_mulquat(quat, quat, tmp);
    mjuu_normvec(quat, 4);
  }

  // sort eigenvalues in decreasing order (bubblesort: 0, 1, 0)
  for (int j = 0; j < 3; j++) {
    int j1 = j%2;
    if (eigval[j1]+kEigEPS < eigval[j1+1]) {
      t = eigval[j1];
      eigval[j1] = eigval[j1+1];
      eigval[j1+1] = t;
      tmp[0] = 0.707106781186548;  // cos(pi/4) = sin(pi/4)
      tmp[1] = tmp[2] = tmp[3] = 0;
      tmp[(j1+2)%3+1] = tmp[0];
      mjuu_mulquat(quat, quat, tmp);
      mjuu_normvec(quat, 4);
    }
  }

  mjuu_quat2mat(eigvec, quat);
  return iter;
}

}  // namespace

// --- Public resolvers ------------------------------------------------------ //
// Lifted from ResolveOrientation (user_objects.cc:241-349): the five authored
// forms + degree + eulerseq, over raw authored values instead of an mjsOrientation.
std::array<double, 4> ResolveOrientation(OrientKind kind, const double* raw,
                                         const OrientContext& ctx) {
  double quat[4] = {1, 0, 0, 0};
  const bool degree = ctx.degree;

  switch (kind) {
    case OrientKind::Quat: {
      quat[0] = raw[0]; quat[1] = raw[1]; quat[2] = raw[2]; quat[3] = raw[3];
      mjuu_normvec(quat, 4);
      break;
    }
    case OrientKind::AxisAngle: {
      double ax[4] = {raw[0], raw[1], raw[2], raw[3]};
      if (degree) ax[3] = ax[3] / 180.0 * mjPI;
      if (mjuu_normvec(ax, 3) < mjEPS) break;
      double ang2 = ax[3] / 2;
      quat[0] = std::cos(ang2); quat[1] = std::sin(ang2) * ax[0];
      quat[2] = std::sin(ang2) * ax[1]; quat[3] = std::sin(ang2) * ax[2];
      break;
    }
    case OrientKind::XYAxes: {
      double a[6]; for (int k = 0; k < 6; ++k) a[k] = raw[k];
      if (mjuu_normvec(a, 3) < mjEPS) break;
      double d = mjuu_dot3(a, a + 3);
      a[3] -= a[0] * d; a[4] -= a[1] * d; a[5] -= a[2] * d;
      if (mjuu_normvec(a + 3, 3) < mjEPS) break;
      double z[3];
      mjuu_crossvec(z, a, a + 3);
      if (mjuu_normvec(z, 3) < mjEPS) break;
      mjuu_frame2quat(quat, a, a + 3, z);
      break;
    }
    case OrientKind::ZAxis: {
      double z[3] = {raw[0], raw[1], raw[2]};
      if (mjuu_normvec(z, 3) < mjEPS) break;
      mjuu_z2quat(quat, z);
      break;
    }
    case OrientKind::Euler: {
      double e[3] = {raw[0], raw[1], raw[2]};
      if (degree) for (int i = 0; i < 3; ++i) e[i] = e[i] / 180.0 * mjPI;
      mjuu_setvec(quat, 1, 0, 0, 0);
      const std::string& seq = ctx.eulerseq;
      for (int i = 0; i < 3 && i < static_cast<int>(seq.size()); ++i) {
        double tmp[4], qrot[4] = {std::cos(e[i] / 2), 0, 0, 0};
        double sa = std::sin(e[i] / 2);
        char ch = seq[i];
        if (ch == 'x' || ch == 'X') qrot[1] = sa;
        else if (ch == 'y' || ch == 'Y') qrot[2] = sa;
        else if (ch == 'z' || ch == 'Z') qrot[3] = sa;
        if (ch == 'x' || ch == 'y' || ch == 'z') mjuu_mulquat(tmp, quat, qrot);
        else mjuu_mulquat(tmp, qrot, quat);
        mjuu_copyvec(quat, tmp, 4);
      }
      mjuu_normvec(quat, 4);
      break;
    }
  }
  return {quat[0], quat[1], quat[2], quat[3]};
}

// Lifted from mjuu_fullInertia (user_util.cc:872-901).
const char* FullInertiaToDiag(const double fullinertia[6], double diag[3],
                              double quat[4]) {
  if (!mjuu_defined(fullinertia[0])) return nullptr;

  double eigval[3], eigvec[9], quattmp[4];
  double full[9] = {
    fullinertia[0], fullinertia[3], fullinertia[4],
    fullinertia[3], fullinertia[1], fullinertia[5],
    fullinertia[4], fullinertia[5], fullinertia[2]
  };
  mjuu_eig3(eigval, eigvec, quattmp, full);

  if (eigval[2] < mjEPS) return "inertia must have positive eigenvalues";

  if (quat) mjuu_copyvec(quat, quattmp, 4);
  if (diag) mjuu_copyvec(diag, eigval, 3);
  return nullptr;
}

}  // namespace ps::core
