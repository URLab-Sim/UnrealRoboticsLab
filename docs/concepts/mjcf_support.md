# MJCF Support

URLab's `MuJoCoSpec` layer delegates XML parsing, compilation, and XML export directly to MuJoCo's native `mjSpec` C API.

Checked against **MuJoCo 4.0.2** (`7252110fb`, `mjVERSION_HEADER 4000002`).

## Coverage and MuJoCo 4 schema changes

Because `UMJModelComponent` builds and compiles a native `mjSpec` and `FMJXmlImporterExporter` parses XML with `mj_parseXMLString` / `mj_saveXMLString`, MJCF support tracks the pinned MuJoCo 4.0.2 runtime:

- **Schema structs and enums:** `Tools/SpecCodegen/generate_ue_spec.py` generates `FMJ*Spec` structs and `EMJ*` enums from MuJoCo 4's `introspect` metadata and `src/xml/mjcf.schema`.
- **MuJoCo 4 `mjcf.schema` updates:** `<composite>` and `<flexvert>` were removed in MuJoCo 4; use `<flexcomp>` and `<replicate>` where applicable. Per-element `flex_matid` is supported.
- **By-reference textures and embedded assets:** MuJoCo 4 supports by-reference textures (`tex_adr == -1`) decoded on demand via `mj_loadTexture`, as well as `mj_embedAssets` (`UMJAssetLibrary::IsTextureByReference`, `LoadTextureFromModel`, `EmbedModelAssets`).
- **Actuator shortcuts:** `UMJActuatorComponent` uses `EMJActuatorShortcut` mapped to MuJoCo's `mjs_setTo*` helpers (`mjs_setToMotor`, `mjs_setToPosition`, `mjs_setToVelocity`, etc.).
- **Attachments:** `UMJAttachComponent` attaches sub-models via `mjs_attach`.

## XML import and export

`FMJXmlImporterExporter` (`Source/URLabEditor/Private/MuJoCoSpec/MJXmlImporterExporter.h`) provides:

- `ImportXMLToActor(Xml, TargetActor, Vfs, OutError)` — parses MJCF into `UMJModelComponent` + `UMJ*Component` hierarchy on an Actor.
- `ImportXMLToBlueprintSCS(Xml, Blueprint, Vfs, OutError)` — imports the hierarchy into a Blueprint's SimpleConstructionScript.
- `ExportActorToXML(SourceActor, OutXml, OutError)` — exports the Actor's `UMJModelComponent` to MJCF XML.

`UMJModelComponent::ExportXML` exposes MuJoCo 4's `mjsCompiler` output flags:

- `savecompiled` (`bSaveCompiled = false`)
- `savecanonical` (`bSaveCanonical = false`)
- `saveinertial` (`bSaveInertial = false`)

All three default to `false` so exported XML preserves the authored representation unless explicitly requested.

## Virtual file system (`mjVFS`)

Referenced meshes, textures, and included XML files are supplied to MuJoCo through `mjVFS` (`FMJVFSHandle` / `UMJAssetLibrary::PopulateVFS` via `mj_addBufferVFS`), keeping runtime and editor compilation self-contained.

## Corpus parity testing

URLab verifies `MuJoCoSpec` round-trip fidelity (XML -> `UMJModelComponent` -> `mjSpec` -> `mj_compile` + 100-step dynamics/sensor comparison against native MuJoCo) with two automation tests in `Source/URLabEditor/Private/MuJoCoSpec/Tests/MJCorpusParityTests.cpp`:

- **`MuJoCoUE.UpstreamMuJoCoCorpusParity`:** Runs offline against all `.xml` models under `third_party/MuJoCo/src/model` and `third_party/MuJoCo/src/test` (with a small allowlist for non-round-trippable `<flexcomp>` `explicitinertial`, external relative includes, plugin geoms, and 1-ULP chaotic multi-contact drift over 100 steps).
- **`MuJoCoUE.MenagerieFullCorpusParity`:** Scans `$URLAB_MENAGERIE_DIR` when set (skips with a warning if unset).

## Related

- [The Component Model](model.md): `UMJModelComponent`, components, and `UMJAssetLibrary`.
- [Generation](generation.md): how `FMJ*Spec` structs and marshalers are generated.
- [Importing Robots](../guides/importing.md): editor import and authoring workflow.
