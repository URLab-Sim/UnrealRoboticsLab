# Debug Visualization

Runtime overlays for inspecting the MuJoCo simulation during PIE. All toggles live on `UMjDebugVisualizer` (auto-created on `AAMjManager`) and are driven by number-row hotkeys from `UMjInputHandler`. Tune the exposed properties live from the Details panel on the Manager actor.

---

## Hotkeys

| Key | Toggle                                            | Purpose |
|-----|---------------------------------------------------|---------|
| `1` | Contact force arrows                              | Yellow arrows at each active contact, scaled by normal force magnitude. |
| `2` | Articulation visual-mesh visibility               | Hide the imported visual meshes (keeps collision/debug overlays). |
| `3` | Articulation collision wireframes                 | Magenta wireframes of every collision geom on articulations. |
| `4` | Joint axes + limit arcs                           | Coloured arrows at each joint showing axis direction and current/limit angle. |
| `5` | Quick-Convert collision wireframes                | Same as `3` but for props spawned via `UMjQuickConvertComponent`. |
| `6` | Cycle body-shader overlay                         | **Off → Island → Instance Segmentation → Semantic Segmentation → Off.** See below. |
| `7` | Tendon/muscle smooth-tube rendering               | Spline-mesh tubes along every tendon path, thickness and colour driven by muscle activation. See below. |
| `P` | Pause/resume the physics thread                   | |
| `R` | Reset simulation state                            | |
| `O` | Toggle orbit and keyframe cameras                 | |
| `F` | Fire all `AMjImpulseLauncher` actors              | |

---

## Body shader overlay (key `6`)

Cycles through three modes plus Off. Repaints every articulation primitive geom, every imported mesh geom, and every `UMjQuickConvertComponent` actor's static-mesh components. Original materials are cached on first apply and restored when the mode returns to Off or PIE ends.

### Island

Colour each body by its **constraint island** — the group of bodies mutually connected through active contacts, joint limits, equality constraints, or friction loss. Implementation mirrors MuJoCo's native visualiser (`engine_vis_visualize.c::addGeomGeoms`) exactly:

- Looks up the body's first DOF via `body_weldid` → `body_dofadr`.
- Reads `mjData.dof_island[dof]`; if ≥ 0 the body is in an active island, seed = `island_dofadr[island]`.
- If `-1` and sleep is enabled, falls back to `tree_dofadr[dof_treeid[dof]]` (optionally passed through `mj_sleepCycle` for sleeping trees so a cluster of bodies resting on each other shares one colour).
- Seed is fed through a Halton sequence (bases 7/3/5) to produce a HSV colour, matching upstream byte-for-byte.

Bodies with no DOFs (worldbody, welded-to-world) appear neutral grey.

### Instance Segmentation

Each body gets a unique Halton-keyed hue mixed from the articulation class hash and the body id. Useful for generating pixel-accurate per-body segmentation masks from a `USceneCapture2D` or a `UMjCamera` — because the overlay paints real materials, it shows up in any standard render pass.

### Semantic Segmentation

Bodies belonging to the same articulation *class* (not instance) share a hue. Two Blueprints instanced from `BP_Humanoid` both read as one colour; a `BP_KitchenTable` reads as a different colour. For Quick-Convert props, the hue is keyed off the first static-mesh asset on the actor so props sharing a mesh group together.

### Sleep modulation

Independent of mode. When `bModulateBySleep` is on (default), sleeping bodies are dimmed and desaturated — `V *= SleepValueScale` (default 0.35), `S *= SleepSaturationScale` (default 0.9). Matches the upstream MuJoCo behaviour; the defaults are slightly more aggressive than upstream so the effect is clearly readable under Unreal's PBR lighting.

Tune the two scales live in the Details panel if sleep needs to read even stronger (lower `SleepValueScale`) or you want fully-grey sleeping bodies (drop `SleepSaturationScale` to 0).

### How it's rendered

One `UMaterialInstanceDynamic` per painted mesh, parented on `/Engine/BasicShapes/BasicShapeMaterial`. At `BeginPlay` the visualiser probes that material for a vector parameter named `Color` / `BaseColor` / `Tint` and uses the first match. No plugin-shipped `.uasset` materials — stays portable across UE versions.

---

## Tendon / muscle rendering (key `7`)

Renders every `<spatial>` / `<fixed>` tendon as a smooth tube. For muscle actuators (dyntype `mjDYN_MUSCLE` with tendon transmission), tube thickness and colour both track the muscle's activation in real time.

### Visual encoding

| Signal          | Visual                                                             |
|-----------------|--------------------------------------------------------------------|
| Muscle activation (0–1) | Radius scales from `0.5×` (relaxed) to `2.0×` (fully contracted). Colour lerps from dark blue to bright red. |
| Limited tendon length (no muscle driver) | Same colour/radius lerp, driven by `(ten_length − range_lo) / (range_hi − range_lo)`. |
| Neutral tendon (no muscle, no limit) | Mid-purple at nominal radius. |

Per-tube colouring is real: each segment owns its own `UMaterialInstanceDynamic` (via `UPrimitiveComponent::CreateDynamicMaterialInstance`) so two muscles on the same arm read distinctly.

### Path construction

Each tendon's path is pulled from `mjData.wrap_xpos` / `wrap_obj` / `ten_wrapadr` / `ten_wrapnum` — this is MuJoCo's post-kinematics tendon path, including tangent points where the tendon touches a wrapping geom. The visualiser:

1. Walks consecutive wrap indices exactly like MuJoCo's own renderer (`engine_vis_visualize.c::addSpatialTendonGeoms`), skipping pulley endpoints (`wrap_obj == -2`).
2. When two consecutive wrap points share the same geom id (i.e. a cylinder or sphere wrap), subdivides the chord with `TendonArcSubdivisions` (default 6) intermediate points along a spherical interpolation about the geom's world position — so the tendon curves *around* the geom instead of jumping straight across, which is what MuJoCo's own viewer skips.
3. Averages tangent directions at every interior wrap point, producing C1-continuous joins between segments.

### How it's rendered

Pool of `USplineMeshComponent`s deforming `/Engine/BasicShapes/Cylinder.Cylinder` along each segment. The cylinder ships with a 50 cm base radius, so `TendonTubeRadius` is converted to an engine scale via `scale = radius_cm / 50`. Components are created lazily, reused frame-to-frame, and hidden (not destroyed) when the tendon count drops.

No physics, no collision, no shadow casting, not selectable in the viewport.

### Tuning

| Property                 | Default | Notes |
|--------------------------|---------|-------|
| `TendonTubeRadius`       | 0.25 cm | Nominal tube radius. Muscle activation scales up to 2×. |
| `TendonArcSubdivisions`  | 6       | Intermediate points per geom-wrap arc. Drop to 0 for MuJoCo's flat chord look. |

---

## Camera interaction (known issue)

Because the hotkey-6 overlay swaps real materials on real `UStaticMeshComponent`s, anything rendering the level — `USceneCaptureComponent2D`, `UMjCamera`, screenshot tools — captures the overlay, not the underlying scene. The overlay is fundamentally a viewport-debug tool.

For persistent camera streams that need pure RGB, depth, or segmentation output regardless of viewport state, use per-camera capture modes instead — see [Camera Capture Modes](camera_capture_modes.md). Those modes are orthogonal to hotkey 6: an `InstanceSegmentation` camera outputs masks even when the viewport overlay is off, and a `Real` camera outputs photoreal RGB even when the viewport is showing islands.

If you specifically need the hotkey-6 overlay active while also capturing RGB through a `UMjCamera`, toggle the overlay off (`6` until Off) before the capture window.

---

## Implementation references

- Hotkey dispatch — [MjInputHandler.cpp](../../Source/URLab/Private/MuJoCo/Input/MjInputHandler.cpp)
- Snapshot capture (physics thread) — [MjDebugVisualizer.cpp:`CaptureDebugData`](../../Source/URLab/Private/MuJoCo/Core/MjDebugVisualizer.cpp)
- Island Halton colour helper — [MjColor.cpp](../../Source/URLab/Private/MuJoCo/Utils/MjColor.cpp) (matches MuJoCo's upstream exactly, covered by `URLab.Color.*` automation tests)
- Upstream reference — MuJoCo's [`engine_vis_visualize.c`](https://github.com/google-deepmind/mujoco/blob/main/src/engine/engine_vis_visualize.c)
