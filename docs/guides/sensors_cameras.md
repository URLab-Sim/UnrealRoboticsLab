# Sensors & Cameras

Read simulation state through `UMJSensorComponent`, capture RGB/depth/segmentation through `URenderCameraComponent`, and light scenes with physical MJCF units.

## Sensors (`UMJSensorComponent`)

Add `UMJSensorComponent` (`Source/URLab/Public/MuJoCoSpec/MJActorComponents.h`) to an Actor with a `UMJModelComponent`:

- Configure `Spec` (`FMJSensorSpec`, including `Type`, `Noise`, `Cutoff`, and `User`) and target references (`ObjRef`, `RefObjRef` via `FMJElementRef`).
- `UMJModelComponent` marshals every `UMJSensorComponent` into `mjSpec` (`mjs_addSensor`), and MuJoCo populates `mjData->sensordata` each step.
- From Python, read sensor and state observations through `SimulationClient` (see [Python & External Control](../python/index.md)).

## Cameras (`UMJCameraComponent` & `URenderCameraComponent`)

- **Authoring:** `UMJCameraComponent` (`Source/URLab/Public/MuJoCoSpec/MJSceneComponents.h`) holds `FMJCameraSpec` (projection, `fovy`, `focal`, `sensorsize`, `intrinsic`, `resolution`) and marshals directly to `mjsCamera`.
- **Neutral rendering:** At runtime, `URenderCameraRig` spawns `URenderCameraComponent` instances (`Source/URLab/Public/Render/RenderCameraComponent.h`) for each camera in the installed scene.

### Capture modes

Requested per-camera through `RenderClient` or `urlab_client.Renderer` (or configured on a continuous camera stream and read via `OutputReader`):

| Mode | Output format | Notes |
|---|---|---|
| `real` | `RGBA8` | Photoreal RGB respecting Unreal lighting and post-process |
| `depth` | `float32` (metres) | Uses `SetNeutralDepthComponents` opaque stand-ins for `M_MuJoCo_Master_Translucent` meshes |
| `segmentation` | `int32` pairs | Object ID and object type per pixel |

For `urlab_client.Renderer` and `RenderClient` examples, see the companion [URLab_Bridge Getting Started](https://github.com/URLab-Sim/URLab_Bridge/blob/main/docs/getting_started.md) and [Typed Render Service](https://github.com/URLab-Sim/URLab_Bridge/blob/main/docs/render_server.md) guides.

## Physical lighting (`UMJLightComponent`)

`UMJLightComponent` (`FMJLightSpec`) and MJCF `<light>` elements use physical light units matching MuJoCo's Filament renderer (`MjRenderSceneAdapter`):

- **Directional lights:** `intensity` is in **lux** (for example, daylight is ~100,000 lux).
- **Punctual lights (spot / point):** `intensity` is in **candela** (small values like `5` will appear nearly black).
- **Image-based lighting:** `<light type="image" texture="sky" intensity="..."/>` drives Unreal's sky light from the referenced skybox texture.

Turn off MuJoCo's default headlight (`<headlight active="0"/>`) when lighting a scene explicitly.
