// Orientation canonicalization through MuJoCo's public resolver.
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

// Which MJCF orientation spelling was authored.
enum class OrientKind { Quat, AxisAngle, XYAxes, ZAxis, Euler };

// Fold an authored orientation spelling into a unit quaternion (w, x, y, z)
// using MuJoCo's public resolver. `raw` holds the authored
// numbers for the kind: Quat -> [w,x,y,z]; AxisAngle -> [ax,ay,az,angle];
// XYAxes -> [x0,x1,x2,y0,y1,y2]; ZAxis -> [z0,z1,z2]; Euler -> [e0,e1,e2].
// A degenerate input yields the identity quaternion, exactly as MuJoCo does.
std::array<double, 4> ResolveOrientation(OrientKind kind, const double* raw,
                                         const OrientContext& ctx);

}  // namespace ps::core

#endif  // PROTOSPEC_CORE_RESOLVE_H
