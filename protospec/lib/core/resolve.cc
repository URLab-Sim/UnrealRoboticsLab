// Orientation normalization through the pinned MuJoCo public API.
// The equivalent API exists at upstream HEAD a79d65c4 (2026-09-27).
#include "resolve.h"

#include <mujoco/mujoco.h>

namespace ps::core {
namespace {
void Copy(double* dest, const double* src, int count) {
  for (int i = 0; i < count; ++i) dest[i] = src[i];
}
}  // namespace

std::array<double, 4> ResolveOrientation(OrientKind kind, const double* raw,
                                         const OrientContext& ctx) {
  double quat[4] = {1, 0, 0, 0};
  if (kind == OrientKind::Quat) {
    Copy(quat, raw, 4);
    mju_normalize4(quat);
    return {quat[0], quat[1], quat[2], quat[3]};
  }

  mjsOrientation alt;
  mjs_defaultOrientation(&alt);
  switch (kind) {
    case OrientKind::AxisAngle:
      alt.type = mjORIENTATION_AXISANGLE;
      Copy(alt.axisangle, raw, 4);
      break;
    case OrientKind::XYAxes:
      alt.type = mjORIENTATION_XYAXES;
      Copy(alt.xyaxes, raw, 6);
      break;
    case OrientKind::ZAxis:
      alt.type = mjORIENTATION_ZAXIS;
      Copy(alt.zaxis, raw, 3);
      break;
    case OrientKind::Euler:
      alt.type = mjORIENTATION_EULER;
      Copy(alt.euler, raw, 3);
      break;
    case OrientKind::Quat:
      break;
  }
  if (mjs_resolveOrientation(quat, ctx.degree, ctx.eulerseq.c_str(), &alt)) {
    return {1, 0, 0, 0};  // Preserve the degenerate-input fallback.
  }
  return {quat[0], quat[1], quat[2], quat[3]};
}
}  // namespace ps::core
