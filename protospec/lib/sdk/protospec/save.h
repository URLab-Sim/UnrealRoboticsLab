// ProtoSpec SDK save surface: first-class model persistence.
//
// The rest of the SDK (sdk.h) is a pure, MuJoCo-free, filesystem-free tree
// library. Saving is deliberately a *separate* opt-in header because it reaches
// out to disk and pulls in the MJCF writer (protospec_io): a consumer that only
// builds and edits trees keeps the pure surface, and only a consumer that wants
// to persist a model includes this and links protospec_sdk_io.
//
//   Save     WriteMjcf(model) -> file on disk.
//   SaveAs   as Save, plus externalize any in-memory assets (mesh/texture/hfield
//            bytes not yet on disk) next to the target, honoring the model's
//            authored <compiler meshdir>. This is the operation that turns a
//            never-saved, in-memory model into a fully on-disk one.
//
// Purity (CDR-14): saving never mutates the model tree. The model is taken by
// const ref; the only mutable output is the asset list a caller hands to
// SaveAs, which is cleared once its bytes live on disk.
//
// ModelAssetDir + WriteAssetFile are the shared asset-externalization
// primitives; ExternalizeAssets is the convenience over the SDK's own asset
// type.
#ifndef PROTOSPEC_SDK_SAVE_H
#define PROTOSPEC_SDK_SAVE_H

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

#include "types.h"

namespace ps::sdk {

namespace mj = ps::mjcf;

// An in-memory asset (mesh/texture/hfield bytes) that a model references by
// basename but that has no file on disk yet -- e.g. a mesh imported into a
// never-saved model. `name` is the basename the model's element names in its
// `file="..."` attribute. Structurally identical to the compile bridge's
// VfsAsset; kept here so the SDK save surface needs no dependency on the bridge.
struct InMemoryAsset {
  std::string name;
  std::vector<std::uint8_t> bytes;
};

// The directory a model's on-disk assets resolve into when the model is saved at
// `xml_path`: the parent directory of `xml_path`, plus the model's authored
// <compiler meshdir> when present (last authored value wins, mirroring the
// compiler). Pure: reads only `model.compilers`. Shared primitive.
std::filesystem::path ModelAssetDir(const mj::Model& model,
                                    const std::filesystem::path& xml_path);

// Write `size` bytes to `dir`/`name`, creating `dir` (and parents) if needed.
// Returns false on any filesystem error. Shared primitive.
bool WriteAssetFile(const std::filesystem::path& dir, const std::string& name,
                    const std::uint8_t* bytes, std::size_t size);

// Externalize every in-memory asset for a model being saved at `xml_path`:
// write each asset's bytes into ModelAssetDir(model, xml_path). Returns the
// number of assets successfully written (== assets.size() on full success).
// The single shared implementation; SaveAs and the studio editor both route
// asset externalization through here (or through the two primitives above).
std::size_t ExternalizeAssets(const mj::Model& model,
                              const std::filesystem::path& xml_path,
                              const std::vector<InMemoryAsset>& assets);

// Serialize `model` to canonical MJCF and write it to `path` (binary, no CRLF
// translation, matching the writer's deterministic bytes). Returns false on any
// filesystem error; the model is never mutated.
bool Save(const mj::Model& model, const std::filesystem::path& path);

// Save `model` to `path` and, when `assets` is non-null and non-empty,
// externalize its in-memory assets next to `path` first (see ExternalizeAssets),
// then clear `assets` -- the bytes now live on disk and the model's basename
// references resolve to files. `assets == nullptr` is exactly Save. Returns true
// only when both the asset writes and the MJCF write succeed. The model tree is
// never mutated (const); the asset list is the sole mutable output.
bool SaveAs(const mj::Model& model, const std::filesystem::path& path,
            std::vector<InMemoryAsset>* assets);

}  // namespace ps::sdk

#endif  // PROTOSPEC_SDK_SAVE_H
