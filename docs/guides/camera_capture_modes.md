# Camera Capture Modes

Every `UMjCamera` has a `CaptureMode` property that decides what the render target contains. One mode per camera — spawn multiple cameras if you need multiple streams (RGB + depth + masks) from the same viewpoint.

| Mode | Output | Render target format | Scene capture source |
|---|---|---|---|
| `Real` (default) | Photoreal RGB. Matches viewport look, respects post-process volumes. | `RTF_RGBA8_SRGB` | `SCS_FinalToneCurveHDR` |
| `Depth` | Linear scene depth in centimetres, single float per pixel. | `RTF_R32f` | `SCS_SceneDepth` |
| `SemanticSegmentation` | Per-actor-class flat tint. Two instances of the same Blueprint share a colour; different Blueprints get different colours. | `RTF_RGBA8` | `SCS_FinalToneCurveHDR` + `PRM_UseShowOnlyList` |
| `InstanceSegmentation` | Per-body unique tint. Every rigid body gets its own hue. | `RTF_RGBA8` | `SCS_FinalToneCurveHDR` + `PRM_UseShowOnlyList` |

---

## Configuring a camera

Set `CaptureMode` on a `UMjCamera` component in the Details panel. For `Depth`, also set `DepthNearCm` (default 10 cm) and `DepthFarCm` (default 10000 cm) — the UI preview and raw depth values are normalised against this range.

`CaptureMode` is read at `SetStreamingEnabled(true)` time. To change mode on a running camera, toggle streaming off and on:

```cpp
Cam->SetStreamingEnabled(false);
Cam->CaptureMode = EMjCameraMode::Depth;
Cam->SetStreamingEnabled(true);
```

Or, from the simulate widget, reselect the articulation (which rebinds all feed entries).

---

## Coexistence — multiple modes in one frame

Seg modes work without swapping materials on the real meshes, so an RGB camera and a seg camera can stream simultaneously in the same frame with no contamination either way.

Under the hood, `UMjDebugVisualizer` maintains a refcounted **sibling-mesh pool** per seg mode. The first seg camera that streams triggers the pool build; additional cameras on the same mode share it; the pool is torn down when the last subscriber unsubscribes.

- Sibling components duplicate every articulation visual mesh and every Quick-Convert static mesh with an unlit tint MID.
- They are `bVisibleInSceneCaptureOnly = true` and also disable reflection captures, sky captures, ray-tracing contribution, distance-field lighting, and dynamic indirect lighting — so secondary lighting passes can't leak a faint tinge into the main viewport.
- Seg cameras use `PrimitiveRenderMode = PRM_UseShowOnlyList` to render only their pool.
- Non-seg URLab cameras (`Real`, `Depth`) refresh their `HiddenComponents` list against the live pools each tick so a late-starting seg camera never contaminates a running RGB stream.

Cost: up to 3 components per body while both seg pools are active (original + instance sibling + semantic sibling). Each view still draws N primitives — per-view draw count is unchanged. Pools are lazy, so RGB-only sessions pay zero.

---

## Simulate widget preview

The simulate widget's camera panel shows every `UMjCamera` on the selected articulation, each rendering its configured mode live. No extra setup — the feed entry inspects `CaptureMode` and picks the right display path:

- `Real` / `SemanticSegmentation` / `InstanceSegmentation` — the render target binds directly as the slate brush resource.
- `Depth` — an R32f RT can't be a slate brush. The entry CPU-copies the R channel each tick into a BGRA `UTexture2D`, mapping `depth ∈ [DepthNearCm, DepthFarCm]` to grayscale `[0, 255]`, and binds that texture instead.

The depth preview path is synchronous (`ReadLinearColorPixels` flushes rendering commands). Fine for debug UI; if you need headless depth capture at full framerate, use the render-target readback directly in Blueprint or C++ rather than the widget preview.

---

## ZMQ broadcast

`bEnableZmqBroadcast` continues to work for `Real`, `SemanticSegmentation`, and `InstanceSegmentation` — the existing BGRA worker transports all three.

`Depth` mode skips ZMQ broadcast (a float-transport path isn't wired yet) and logs a one-shot warning. Track the depth-stream follow-up if you need it.

---

## Known v1 limitations

- **Seg tints are lit-shaded, not flat.** Seg cameras use `SCS_FinalToneCurveHDR` because `BasicShapeMaterial`'s `Color` parameter isn't wired directly to the BaseColor G-buffer. Scene lighting therefore modulates the tint. Good enough for visual debugging; not pixel-exact for ground-truth segmentation masks. The cleanest fix is a plugin-shipped unlit parent material — on the roadmap.
- **Mode changes mid-PIE require toggling streaming off/on.** The render target format is mode-specific (`RTF_RGBA8` vs `RTF_R32f`), and live reconfiguration isn't wired.
- **Runtime articulation spawn.** The sibling pool is built on first seg subscriber. Articulations or Quick-Convert props that spawn *after* a seg camera is already streaming won't appear in its output until the pool is rebuilt (toggle that seg camera's streaming off/on to force a rebuild).
- **Third-party `USceneCaptureComponent2D`.** User-authored scene captures outside URLab's management won't automatically hide siblings — they'll see them as regular primitives (since siblings are `bVisibleInSceneCaptureOnly = true`, they *are* visible to scene captures by definition). Add the sibling pool to that component's `HiddenComponents` manually if this is a problem.

---

## Implementation references

- Mode enum + per-camera state — [MjCameraTypes.h](../../Source/URLab/Public/MuJoCo/Components/Sensors/MjCameraTypes.h), [MjCamera.h](../../Source/URLab/Public/MuJoCo/Components/Sensors/MjCamera.h)
- Per-mode render target + scene-capture setup — [MjCamera.cpp::SetupRenderTarget](../../Source/URLab/Private/MuJoCo/Components/Sensors/MjCamera.cpp)
- Pool lifecycle — [MjDebugVisualizer.cpp::AcquireSegPool / BuildSegPool / DestroySegPool](../../Source/URLab/Private/MuJoCo/Core/MjDebugVisualizer.cpp)
- Depth preview — [MjCameraFeedEntry.cpp::UpdateDepthPreview](../../Source/URLab/Private/UI/MjCameraFeedEntry.cpp)
- Automation coverage — `URLab.Camera.*` in [MjCameraTests.cpp](../../Source/URLabEditor/Private/Tests/MjCameraTests.cpp)
