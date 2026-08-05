// Orientation + inertia canonicalization resolvers (Q-ORIENT / Q-INERTIA).
//
// ProtoSpec stores orientation as a single canonical unit quaternion and inertia
// as diaginertia + iquat (docs/plan_canonicalization.md, Wave A). The MJCF reader
// accepts every authored spelling (quat/euler/axisangle/xyaxes/zaxis;
// diaginertia/fullinertia) and folds it here, at parse end, against the effective
// compiler context. Placing the fold in this MuJoCo-free module (plan.md Section 3
// core/resolve) keeps the reader (protospec_io) MuJoCo-free while sharing the
// exact math MuJoCo compiles: the resolvers are lifted verbatim from the vendored
// tree (see resolve.cc for provenance and snapshots/lifted_code.json).
#ifndef PROTOSPEC_CORE_RESOLVE_H
#define PROTOSPEC_CORE_RESOLVE_H

#include <array>
#include <string>

namespace ps::core {

// The compiler context an orientation fold consumes. Folded document-order
// independently from Model.compilers (later authored attributes win), matching
// MuJoCo's accumulate-into-one-spec behavior. Defaults: degrees, eulerseq "xyz".
struct OrientContext {
  bool degree = true;            // compiler.angle == "degree"
  std::string eulerseq = "xyz";  // compiler.eulerseq (per-character intrinsic/
                                 //   extrinsic, lower/upper case)
};

// Which MJCF orientation spelling was authored (Q-ORIENT).
enum class OrientKind { Quat, AxisAngle, XYAxes, ZAxis, Euler };

// Fold an authored orientation spelling into a unit quaternion (w, x, y, z),
// lifted from ResolveOrientation (user_objects.cc). `raw` holds the authored
// numbers for the kind: Quat -> [w,x,y,z]; AxisAngle -> [ax,ay,az,angle];
// XYAxes -> [x0,x1,x2,y0,y1,y2]; ZAxis -> [z0,z1,z2]; Euler -> [e0,e1,e2].
// A degenerate input yields the identity quaternion, exactly as MuJoCo does.
std::array<double, 4> ResolveOrientation(OrientKind kind, const double* raw,
                                         const OrientContext& ctx);

// Eigendecompose a symmetric full inertia matrix (fullinertia = [xx,yy,zz,xy,
// xz,yz]) into principal moments (diag, descending) and the quaternion of its
// principal frame, lifted from mjuu_fullInertia (user_util.cc). Returns nullptr
// on success, or a MuJoCo-verbatim error string for a non-positive inertia.
const char* FullInertiaToDiag(const double fullinertia[6], double diag[3],
                              double quat[4]);

}  // namespace ps::core

#endif  // PROTOSPEC_CORE_RESOLVE_H
