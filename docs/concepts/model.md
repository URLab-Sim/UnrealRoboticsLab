# The Component Model

In URLab, a robot or scene authored in Unreal is represented by a `UMJModelComponent` paired with a hierarchy of `MuJoCoSpec` components on an Actor or Blueprint. When compiled, `UMJModelComponent` marshals those components directly into a MuJoCo `mjSpec` and compiles `mjModel` + `mjData` via `mj_compile` (or updates them in place via `mj_recompile`).

For how the underlying structs, enums, and marshalers are generated from MuJoCo 4, see [Generation](generation.md).

## `UMJModelComponent`

`UMJModelComponent` (`Source/URLab/Public/MuJoCoSpec/MJModelComponent.h`) is a `UActorComponent` that owns the MuJoCo lifecycle for an Actor's component hierarchy:

- **RAII handles:** `FMJSpecHandle` (`mjSpec*`), `FMJModelHandle` (`mjModel*`), `FMJDataHandle` (`mjData*`), and `FMJVFSHandle` (`mjVFS*`).
- **Configuration:** `ModelName`, `bCompilerDegree`, `AssetLibrary` (`UMJAssetLibrary*`), and `DefaultClasses` (`TArray<FMJDefaultClassEntry>`).
- **Lifecycle methods:**
  - `BuildSpecFromActor(OutError)` — walks the Actor's spatial and non-spatial `MuJoCoSpec` components into a fresh `mjSpec`.
  - `CompileModel(OutError)` — compiles `mjSpec` into `mjModel` + `mjData` via `mj_compile`.
  - `RecompileModel(OutError)` — rebuilds `mjSpec` and calls `mj_recompile` in place, preserving `qpos` and `qvel`.
  - `StepSimulation(Dt)` and `SyncTransformsFromData()` — advances physics and updates `UMJBodyComponent` transforms from `mjData`.
  - `ExportXML(OutXml, OutError, bSaveCompiled=false, bSaveCanonical=false, bSaveInertial=false)` — exports the spec to MJCF XML using MuJoCo 4 `mjsCompiler` flags (`savecompiled`, `savecanonical`, `saveinertial`).

## Spatial and non-spatial components

Every authored MuJoCo element maps to a clean `UActorComponent` or `USceneComponent` subclass under `Source/URLab/Public/MuJoCoSpec/`:

| Component | Base | Role |
|---|---|---|
| `UMJBodyComponent` | `UMJSceneComponent` | Rigid body (`<body>`) with inertial/runtime body ID |
| `UMJFrameComponent` | `UMJSceneComponent` | Coordinate frame (`<frame>`) |
| `UMJGeomComponent` | `UMJSceneComponent` | Collision/visual geometry (`<geom>`), including `fromto` in `FMJGeomSpec` |
| `UMJJointComponent` | `UMJSceneComponent` | Joint (`<joint>`) |
| `UMJSiteComponent` | `UMJSceneComponent` | Site marker (`<site>`) |
| `UMJCameraComponent` | `UMJSceneComponent` | Camera (`<camera>`) |
| `UMJLightComponent` | `UMJSceneComponent` | Light (`<light>`) |
| `UMJAttachComponent` | `UMJSceneComponent` | Attaches a sub-model spec via MuJoCo 4 `mjs_attach` |
| `UMJActuatorComponent` | `UMJActorComponent` | Actuator (`<actuator>`), using `EMJActuatorShortcut` -> `mjs_setTo*` |
| `UMJSensorComponent` | `UMJActorComponent` | Sensor (`<sensor>`) |
| `UMJTendonComponent` | `UMJActorComponent` | Spatial or fixed tendon (`<tendon>`) |

Cross-element references (such as a geom's material or mesh, or an actuator's target) use `FMJElementRef` (`Source/URLab/Public/MuJoCoSpec/MJElementRef.h`).

## `UMJAssetLibrary` and MuJoCo 4 VFS

`UMJAssetLibrary` (`Source/URLab/Public/MuJoCoSpec/MJAssetLibrary.h`) is a `UDataAsset` storing `UStaticMesh` and `UTexture2D` references alongside `RawBuffers` (`TMap<FString, FMJRawAssetBuffer>`) keyed by VFS filename.

- `PopulateVFS(mjVFS*)` registers raw payloads via `mj_addBufferVFS`.
- `IsTextureByReference(M, TexId)` checks `tex_adr == -1` for MuJoCo 4 by-reference textures.
- `LoadTextureFromModel(...)` decodes textures (including by-reference textures) via `mj_loadTexture`.
- `EmbedModelAssets(M, OutEmbedded, OutError)` creates an embedded `mjModel` via `mj_embedAssets`.

## Related

- [Generation](generation.md): how `FMJ*Spec` structs and marshalers are generated.
- [Architecture](../architecture/intended_design.md): runtime simulation, neutral renderer, and transport boundaries.
- [Articulations](../guides/articulations.md): authoring component trees in Unreal.
- [MJCF Support](mjcf_support.md): MuJoCo 4.0.2 schema coverage and XML import/export.
