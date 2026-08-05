#include "sizes.h"

#include <variant>

namespace ps::mjcf::validate {
namespace {

// nq/nv per joint type (user_objects.cc:3114-3140).
void AddJoint(JointType t, ModelSizes& s) {
  switch (t) {
    case JointType::free:  s.nq += 7; s.nv += 6; break;
    case JointType::ball:  s.nq += 4; s.nv += 3; break;
    case JointType::slide: s.nq += 1; s.nv += 1; break;
    case JointType::hinge: s.nq += 1; s.nv += 1; break;
  }
}

// Effective actuator activation dimension (user_objects.cc:7204-7227): an
// explicit non-negative actdim wins; otherwise 0 for none/dcmotor, 1 otherwise.
int ActDim(DynType dyn, const ps::opt<int32_t>& actdim) {
  if (actdim && *actdim >= 0) return *actdim;
  return (dyn != DynType::none && dyn != DynType::dcmotor) ? 1 : 0;
}

void WalkBodySubtree(const std::vector<BodyChildAny>& subtree, ModelSizes& s);

// A body-context container (Body/Frame/Replicate) holds an ordered subtree;
// Frame/Replicate are transparent groupings, Body is a real kinematic body.
void CountBody(const Body& b, ModelSizes& s) {
  if (b.mocap && *b.mocap) ++s.nmocap;
  WalkBodySubtree(b.subtree, s);
}

void WalkBodySubtree(const std::vector<BodyChildAny>& subtree, ModelSizes& s) {
  for (const auto& item : subtree) {
    switch (item.kind()) {
      case BodyChildAny::Kind::Joint: {
        const auto& j = std::get<std::unique_ptr<Joint>>(item.node);
        AddJoint(j->type ? *j->type : JointType::hinge, s);
        break;
      }
      case BodyChildAny::Kind::FreeJoint:
        AddJoint(JointType::free, s);
        break;
      case BodyChildAny::Kind::Body:
        CountBody(*std::get<std::unique_ptr<Body>>(item.node), s);
        break;
      case BodyChildAny::Kind::Frame:
        WalkBodySubtree(std::get<std::unique_ptr<Frame>>(item.node)->subtree, s);
        break;
      case BodyChildAny::Kind::Replicate:
        // MuJoCo clones the subtree `count` times at compile; the pre-compile
        // count is a lower bound.
        s.exact = false;
        WalkBodySubtree(
            std::get<std::unique_ptr<Replicate>>(item.node)->subtree, s);
        break;
      case BodyChildAny::Kind::Composite:
      case BodyChildAny::Kind::Flexcomp:
      case BodyChildAny::Kind::Attach:
        // Expanded into bodies/joints by MuJoCo at compile (DR-7).
        s.exact = false;
        break;
      case BodyChildAny::Kind::Geom:
      case BodyChildAny::Kind::Site:
      case BodyChildAny::Kind::Camera:
      case BodyChildAny::Kind::Light:
      case BodyChildAny::Kind::PluginRef:
        break;
    }
  }
}

void CountActuator(const ActuatorAny& a, ModelSizes& s) {
  ++s.nu;
  switch (a.kind()) {
    case ActuatorAny::Kind::ActuatorGeneral: {
      const auto& g = std::get<std::unique_ptr<ActuatorGeneral>>(a.node);
      s.na += ActDim(g->dyntype ? *g->dyntype : DynType::none, g->actdim);
      break;
    }
    case ActuatorAny::Kind::IntVelocity:
      s.na += 1;  // dyntype integrator
      break;
    case ActuatorAny::Kind::Muscle:
      s.na += 1;  // dyntype muscle
      break;
    case ActuatorAny::Kind::Pid: {
      // mjs_setToPID elects mjDYN_PID when ki or slewmax is set (user_api.cc:
      // 1352-1390); the controller then carries one state per set slot, in
      // order [slew, integral] (user_objects.cc:7237-7246).
      const auto& p = std::get<std::unique_ptr<Pid>>(a.node);
      s.na += (p->ki && *p->ki > 0) + (p->slewmax && *p->slewmax > 0);
      break;
    }
    case ActuatorAny::Kind::ActuatorPlugin:
      s.na_exact = false;  // plugin-defined activation, opaque here
      break;
    case ActuatorAny::Kind::Motor:
    case ActuatorAny::Kind::Position:
    case ActuatorAny::Kind::Velocity:
    case ActuatorAny::Kind::OrientationActuator:
    case ActuatorAny::Kind::Damper:
    case ActuatorAny::Kind::Cylinder:
    case ActuatorAny::Kind::Adhesion:
    case ActuatorAny::Kind::DcMotor:
      // dyntype none/dcmotor -> zero activation state.
      break;
  }
}

}  // namespace

ModelSizes ComputeSizes(const Model& model) {
  ModelSizes s;
  for (const auto& b : model.worldbody) {
    if (b) CountBody(*b, s);
  }
  if (!model.deformables.empty()) {
    for (const auto& d : model.deformables) {
      if (d && !d->flexs.empty()) s.exact = false;
    }
  }
  for (const auto& act : model.actuators) {
    if (!act) continue;
    for (const auto& item : act->actuators) CountActuator(item, s);
  }
  return s;
}

}  // namespace ps::mjcf::validate
