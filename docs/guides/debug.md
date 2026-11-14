# Debug Visualization

URLab provides editor viewport visualizers for `MuJoCoSpec` components and configurable overlay masks on render/viewer servers.

## Editor viewport visualizers (`MJSpecVisualizers`)

When editing a `UMJModelComponent` Actor or Blueprint in the Unreal Editor (`Source/URLabEditor/Private/MuJoCoSpec/MJSpecVisualizers.cpp`), three component visualizers draw in the viewport:

- **`UMJJointComponent` (`FMJJointVisualizer`):** Draws a directional arrow along `Spec.Axis` plus a type glyph (circle for `Hinge`, back-line for `Slide`, wireframe sphere for `Ball`).
- **`UMJSiteComponent` (`FMJSiteVisualizer`):** Draws local RGB axes and a wireframe shape (`Box`, `Capsule`, `Cylinder`, or `Sphere`) using `Spec.Size` and `Spec.Rgba`.
- **`UMJFrameComponent` (`FMJFrameVisualizer`):** Draws local RGB coordinate axes.

## Render & viewer overlays

For render/viewer processes (`-URLabWorkflow=render|viewer`), overlay flags are configured at launch via `-URLabScene` (`Source/URLab/Public/MuJoCo/Render/MjLauncherFlags.h`):

- `-URLabScene=overlay=<mask>`: `mjVIS_*` bitmask (decimal or `0x`-hex) seeding the `mj.RenderOverlayMask` console variable.
- `-URLabScene=maxcontacts=<n>`: Caps how many contact points are serialized per frame (`0` or omitted = uncapped).

Overlays use `/UnrealRoboticsLab/Materials/M_MjOverlay`. If a packaged render server is missing plugin materials, ensure `/UnrealRoboticsLab/Materials` is included in `DirectoriesToAlwaysCook` (automated by `Scripts/package_render_server_linux.sh`). See [Packaging a render server](render_server_packaging.md) and [Workflow launch flags](../reference/render_server_flags.md).
