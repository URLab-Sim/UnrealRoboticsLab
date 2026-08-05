// ProtoSpec public API — validation.
//
// The `ps::mjcf::validate` surface: Validate runs three tiers over a const
// Model -- Structural (required fields, enum legality, arity, cardinality),
// Referential (typed Ref<T> resolution, name uniqueness, default-class
// resolution), Semantic (MuJoCo-grounded lint). It never mutates the tree and
// returns a flat list of provenance-cited Diagnostics; tier selection is a
// bitmask (kTierStructural / kTierReferential / kTierSemantic / kAllTiers).
#ifndef PROTOSPEC_PUBLIC_VALIDATE_H
#define PROTOSPEC_PUBLIC_VALIDATE_H

#include "protospec/model.h"

#include "../../validate/validate.h"  // ps::mjcf::validate::{Validate, Tier, Severity, ...}; ps::Diagnostic

#endif  // PROTOSPEC_PUBLIC_VALIDATE_H
