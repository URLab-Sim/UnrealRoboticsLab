// ProtoSpec validation: three-tier, provenance-cited, never-mutating (plan
// Section 9). Validation runs over a const Model and returns a flat list of
// structured Diagnostics; it never edits the tree (DR-4, DR-8, DR-9).
//
//   Tier 1  Structural  -- driven entirely by the generated reflect/visit
//                          tables: required fields present, enum values legal,
//                          variable-arity bounds, variant well-formedness,
//                          child-list cardinality. No per-element hand code.
//   Tier 2  Referential -- every typed Ref<T> resolves (through union targets
//                          and the implicit "world"/"main" symbols), names are
//                          unique per MuJoCo object namespace (empties allowed),
//                          default class references resolve; dangling
//                          dclass/childclass is reported here, never by the
//                          reader (the reader stays resolution-free).
//   Tier 3  Semantic     -- lint grounded in MuJoCo's compiler: keyframe vector
//                          lengths vs computed model sizes, range-without-limited
//                          under autolimits=false, hinge/slide zero axis, mocap
//                          bodies must be static world children, nuser_* vs user
//                          array lengths, plugin config/ordering rules.
//
// Every Diagnostic carries {tier, severity, message, SourceLoc, element path}.
// Tier 1 and 2 findings are Errors; tier 3 findings are Warnings (ProtoSpec is
// an authoring-level model, so semantic lint never blocks a still-editable
// tree). Validation is a standalone pass, not part of Compile: the front-ends
// that drive a model to compile (the studio editor, the Python bindings) run
// tiers 1-2 first and surface the diagnostics (plan Section 9). The bridge's
// Compile() itself does not validate -- it assumes a validated tree and lets
// MuJoCo reject any residual structural error.
#ifndef PROTOSPEC_VALIDATE_VALIDATE_H
#define PROTOSPEC_VALIDATE_VALIDATE_H

#include <string>
#include <vector>

#include "protospec/core.h"
#include "protospec/diag.h"
#include "types.h"

namespace ps::mjcf::validate {

// Validation findings are the shared ps::Diagnostic (protospec/diag.h): source
// "validate", `tier` carries the structural/referential/semantic tier, and the
// element path ("model/worldbody/body[torso]/geom[shin]", name in brackets when
// authored else a #index) rides in `tag`. Tier/Severity are aliased here so the
// tier taxonomy keeps its home in the validator's namespace.
using Tier = ps::Diagnostic::Tier;
using Severity = ps::Diagnostic::Severity;

// Tier selection bitmask. Combine the flags; kAllTiers runs everything.
using TierMask = unsigned;
inline constexpr TierMask kTierStructural = 1u << 0;
inline constexpr TierMask kTierReferential = 1u << 1;
inline constexpr TierMask kTierSemantic = 1u << 2;
inline constexpr TierMask kAllTiers =
    kTierStructural | kTierReferential | kTierSemantic;

// Validate `model`, running the tiers selected by `tiers`. Const in, vector
// out: the model is never mutated. Diagnostics are returned in traversal order.
std::vector<ps::Diagnostic> Validate(const Model& model,
                                     TierMask tiers = kAllTiers);

}  // namespace ps::mjcf::validate

#endif  // PROTOSPEC_VALIDATE_VALIDATE_H
