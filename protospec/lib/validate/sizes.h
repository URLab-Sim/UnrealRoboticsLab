// Model-size computation from the ProtoSpec tree (plan Q-KEY, DR-4).
//
// Predicts the pre-compile sizes MuJoCo's compiler derives, so validation can
// check keyframe vector lengths without a compile. Grounded field-for-field in
// the vendored MuJoCo compiler:
//
//   nq/nv per joint type  free 7/6, ball 4/3, slide 1/1, hinge 1/1
//                         (user_objects.cc:3114-3140 mjCJoint::nq/nv), summed
//                         over all joints (user_model.cc:2133-2138 SetSizes).
//   nu = number of actuators                     (user_model.cc:2163-2166, :364)
//   na = sum over actuators of actdim            (user_model.cc:2165, :3339)
//        actdim from dyntype: unset -> (dyntype != none && dyntype != dcmotor)
//        i.e. 0 for none/dcmotor, 1 otherwise; explicit actdim honored
//                                                (user_objects.cc:7204-7227)
//   nmocap = count of bodies with mocap=true     (user_model.cc:5156-5163)
//
// `exact` is cleared when the tree contains procedural elements MuJoCo expands
// at compile (composite/flexcomp/replicate/attach/flex) or plugin actuators
// with opaque activation dimension -- in those cases the pre-compile counts are
// a lower bound, not the compiled totals, and size-dependent lint is suppressed.
#ifndef PROTOSPEC_VALIDATE_SIZES_H
#define PROTOSPEC_VALIDATE_SIZES_H

#include "types.h"

namespace ps::mjcf::validate {

struct ModelSizes {
  int nq = 0;
  int nv = 0;
  int nu = 0;
  int na = 0;
  int nmocap = 0;
  bool exact = true;       // false: macros/flex present, counts are a lower bound
  bool na_exact = true;    // false: plugin actuator present, na unreliable
};

ModelSizes ComputeSizes(const Model& model);

}  // namespace ps::mjcf::validate

#endif  // PROTOSPEC_VALIDATE_SIZES_H
