# Geometry & Collision

Define collision and visual shapes with `UMJGeomComponent`, convert static meshes or landscapes into MuJoCo bodies, and render deformables and translucent surfaces.

## Collision & visual shapes (`UMJGeomComponent`)

Add `UMJGeomComponent` under a `UMJBodyComponent` (or `UMJFrameComponent`) and configure `Spec` (`FMJGeomSpec`):

- **Analytic primitives:** `Box`, `Sphere`, `Capsule`, `Cylinder`, `Ellipsoid`, and `Plane` provide fast, exact collision.
- **From-To endpoints:** Enable **From-To Endpoints** (`bUseFromTo`) in the Details panel (`FMJGeomDetails`) to author capsules, cylinders, or boxes from `From` and `To` endpoints instead of pose + size.
- **Meshes (`EMJGeom::Mesh`):** Reference a mesh asset via `MeshRef` (`FMJElementRef`) and `UMJAssetLibrary`. MuJoCo computes convex hulls for mesh collision; for concave meshes, decompose into convex pieces before authoring.
- **Material & appearance:** Assign `MaterialRef` (`FMJElementRef`) or `Spec.Rgba`. By convention, group `2` is used for visual-only meshes (`contype=0`, `conaffinity=0`) and group `3` for collision-only meshes.

Inertial properties (`mass`, `pos`, `quat`, `diaginertia`, `fullinertia`) live directly on `UMJBodyComponent::Spec` (`FMJBodySpec`), or are inferred by MuJoCo from geom density and volume.

## Quick Convert & Heightfields

- **`UMjQuickConvertComponent`** (`Source/URLab/Public/MuJoCo/Convert/MjQuickConvertComponent.h`) turns an Unreal Static Mesh Actor into a MuJoCo physics body (with `ComplexMeshRequired` and `CoACDThreshold` options for CoACD convex decomposition).
- **`AMjHeightfieldActor`** (`Source/URLab/Public/MuJoCo/Convert/AMjHeightfieldActor.h`) samples Unreal Landscape/terrain geometry into a MuJoCo heightfield (`ElevationTraceChannel`, `TraceWhitelist`).

## Contact filtering

Control which geoms collide using standard MuJoCo `mjSpec` fields:

- **Bitmasks:** `Contype` and `Conaffinity` on `FMJGeomSpec` (two geoms collide when `(contype1 & conaffinity2) || (contype2 & conaffinity1)`).
- **Explicit pairs & exclusions:** `<pair>` and `<exclude>` elements in MJCF are preserved through `mjSpec` compilation (`UMJModelComponent`).

## Deformable bodies (`<flexcomp>`)

MuJoCo 4 uses `<flexcomp>` (and `<replicate>`) for deformable ropes, cloth, and volumetric solids (`<composite>` and `<flexvert>` were removed in MuJoCo 4).

- **Native compilation:** `<flexcomp>` elements in imported MJCF are compiled directly by MuJoCo 4's `mjSpec`.
- **Multi-material flex rendering:** URLab supports per-element `flex_matid` (`MjRenderSceneAdapter` / `MjRenderResourceShim`), allowing individual flex elements to resolve distinct materials.
- **Anti-aliasing note:** CPU-deformed flex meshes do not emit motion vectors, so Temporal Anti-Aliasing can smear fast deformations. Set **Project Settings > Rendering > Anti-Aliasing Method** to **FXAA** (`r.AntiAliasingMethod=1`) for flex-heavy scenes.

## Translucent surfaces & depth capture

Geoms and meshes with alpha `< 1.0` render in RGB using `/UnrealRoboticsLab/Materials/M_MuJoCo_Master_Translucent`. Because translucent Unreal materials do not normally write to the scene depth buffer, `URenderCameraComponent::SetNeutralDepthComponents` pairs translucent meshes with hidden opaque depth stand-in components during depth capture so depth frames remain accurate.

## Next steps

- [Articulations](articulations.md) for building `MuJoCoSpec` component trees and editor visualizers.
- [Sensors & Cameras](sensors_cameras.md) for camera capture modes and physical lighting.
