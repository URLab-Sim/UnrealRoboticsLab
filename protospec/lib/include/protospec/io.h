// ProtoSpec public API — MJCF read / write.
//
// The `ps::mjcf::io` surface: ParseMjcfFile / ParseMjcfString turn an MJCF
// document into a `ps::mjcf::Model`; WriteMjcf serializes a Model back to a
// canonical MJCF string. ParseResult carries the model on success or structured
// Diagnostics (malformed vs unsupported-element) otherwise. This is the one
// module that links tinyxml2; the object model itself stays dependency-free.
//
// To persist a Model to disk with asset externalization, see the SDK save
// surface <protospec/save.h> (Save / SaveAs).
#ifndef PROTOSPEC_PUBLIC_IO_H
#define PROTOSPEC_PUBLIC_IO_H

#include "protospec/model.h"

#include "../../io/mjcf.h"  // ps::mjcf::io::{ParseMjcf*, WriteMjcf, ParseResult}; ps::Diagnostic

#endif  // PROTOSPEC_PUBLIC_IO_H
