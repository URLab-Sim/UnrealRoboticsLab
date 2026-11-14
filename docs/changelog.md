# Changelog

Notable changes to UnrealRoboticsLab, newest first.

URLab is in **beta**: the public API and on-disk formats are close to stable,
but a milestone can still include breaking changes. The last alpha is preserved
on the `alpha` branch and the `v0.5.0-alpha` tag.

## MuJoCo 4.0.2 & `MuJoCoSpec` cutover

Clean-slate cutover to **MuJoCo 4.0.2** (`7252110fb`, `mjVERSION_HEADER 4000002`)
and the `MuJoCoSpec` (`mjSpec`) authoring layer.

### Added

- **MuJoCo 4.0.2 support:**
  - By-reference texture handling (`tex_adr == -1`, `mj_loadTexture`, and
    `mj_embedAssets` in `UMJAssetLibrary`).
  - Per-element `flex_matid` support and updated `mjcf.schema` handling
    (`<composite>` and `<flexvert>` removed in MuJoCo 4).
  - `mjsCompiler` export flags (`savecompiled`, `savecanonical`, `saveinertial`)
    exposed on `UMJModelComponent::ExportXML` (all `false` by default).
- **Clean-slate `MuJoCoSpec` (`mjSpec`) components:**
  - `UMJModelComponent` owning `mjSpec*`, `mjModel*`, `mjData*`, `mjVFS*`,
    `DefaultClasses`, and `UMJAssetLibrary` (`mj_addBufferVFS`).
  - Spatial components: `UMJBodyComponent`, `UMJFrameComponent`,
    `UMJGeomComponent` (including `fromto`), `UMJJointComponent`,
    `UMJSiteComponent`, `UMJCameraComponent`, `UMJLightComponent`, and
    `UMJAttachComponent` (`mjs_attach`).
  - Non-spatial components: `UMJActuatorComponent` (`EMJActuatorShortcut` ->
    `mjs_setTo*`), `UMJSensorComponent`, and `UMJTendonComponent`.
  - Editor XML import/export via `FMJXmlImporterExporter`
    (`ImportXMLToActor`, `ImportXMLToBlueprintSCS`, `ExportActorToXML`).
- **`SpecCodegen` pipeline:** `Tools/SpecCodegen/generate_ue_spec.py` generates
  `MJSpecEnums.h`, `MJSpecStructs.h`, `MJSpecMarshal.h`, and
  `MJSpecMarshal.cpp` from MuJoCo 4 `introspect` and `mjcf.schema`, with a
  `--check` drift gate in `Scripts/build_and_test_linux.sh`.

### Removed

- Retired the legacy schema-profile toolchain and per-element generated UCLASS authoring layer.

## v0.6.0-beta (2026-08-11)

Early beta milestone introducing typed service clients (`SimulationClient`,
`RenderClient`, `ViewerClient`, `EditorClient`), MuJoCo engine plugin
packaging, and `control_owner` RPC authority fixes.

**Breaking: MjArticulation assets from any alpha will not open.** Stay on the
`alpha` branch (`v0.5.0-alpha`) if you need legacy alpha Blueprints.

## v0.5.0-alpha

Final alpha release (preserved on the `alpha` branch).
