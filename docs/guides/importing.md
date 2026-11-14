# Importing and authoring MuJoCo models

URLab's `MuJoCoSpec` layer imports MJCF `.xml` models directly into `UMJModelComponent` and `UMJ*Component` hierarchies backed by MuJoCo 4.0.2's `mjSpec` C API.

`FMJXmlImporterExporter` (`Source/URLabEditor/Private/MuJoCoSpec/MJXmlImporterExporter.h`) provides the editor import and export entry points:

- `ImportXMLToActor(Xml, TargetActor, Vfs, OutError)`
- `ImportXMLToBlueprintSCS(Xml, Blueprint, Vfs, OutError)`
- `ExportActorToXML(SourceActor, OutXml, OutError)`

The Content Browser import action (`MujocoImportFactory`) and the editor-only `editor_import_xml` RPC (`MjEditorOpHandlers`) both import MJCF into Unreal Blueprints/Actors. The `editor_import_xml` RPC accepts an XML `path` and optional `force_reimport`, and is unavailable in packaged games.

## Import an XML model

1. Drag or import an `.xml` file through the Unreal Content Browser (or invoke `editor_import_xml`).
2. Inspect the resulting `UMJModelComponent` and its child components (`UMJBodyComponent`, `UMJFrameComponent`, `UMJGeomComponent`, `UMJJointComponent`, `UMJSiteComponent`, `UMJCameraComponent`, `UMJLightComponent`, `UMJAttachComponent`, `UMJActuatorComponent`, `UMJSensorComponent`, `UMJTendonComponent`).
3. Referenced mesh, texture, and include payloads are stored in `UMJAssetLibrary::RawBuffers` and fed into MuJoCo's `mjVFS` via `mj_addBufferVFS`. MuJoCo 4 by-reference textures (`tex_adr == -1`) are decoded via `mj_loadTexture` and can be embedded via `mj_embedAssets`.

## Author a scene in Unreal

You can assemble or edit a model directly in Unreal by adding a `UMJModelComponent` to an Actor or Blueprint and attaching `MuJoCoSpec` components:

- Spatial elements (`UMJBodyComponent`, `UMJFrameComponent`, `UMJGeomComponent`, `UMJJointComponent`, `UMJSiteComponent`, `UMJCameraComponent`, `UMJLightComponent`, `UMJAttachComponent`) derive from `UMJSceneComponent`.
- Non-spatial elements (`UMJActuatorComponent`, `UMJSensorComponent`, `UMJTendonComponent`) derive from `UMJActorComponent`.
- `UMJGeomComponent` holds `FMJGeomSpec` (including `fromto` endpoints) and `FMJElementRef` material/mesh references.
- `UMJActuatorComponent` uses `EMJActuatorShortcut` mapped to MuJoCo's `mjs_setTo*` helpers.
- `UMJModelComponent::DefaultClasses` (`TArray<FMJDefaultClassEntry>`) configures MJCF `<default>` classes.

## Compile, step, and export

- `UMJModelComponent::BuildSpecFromActor(OutError)` constructs the `mjSpec` from the Actor's component tree.
- `UMJModelComponent::CompileModel(OutError)` compiles `mjSpec` -> `mjModel` + `mjData` (`mj_compile`).
- `UMJModelComponent::RecompileModel(OutError)` rebuilds `mjSpec` and calls `mj_recompile` in place, preserving `qpos` and `qvel`.
- `UMJModelComponent::ExportXML(OutXml, OutError, bSaveCompiled=false, bSaveCanonical=false, bSaveInertial=false)` exports MJCF XML with `mjsCompiler` flags (`savecompiled`, `savecanonical`, `saveinertial`) defaulting to `false`.

See [The Component Model](../concepts/model.md) and [MJCF Support](../concepts/mjcf_support.md) for details on `UMJAssetLibrary` and MuJoCo 4.0.2 schema coverage.
