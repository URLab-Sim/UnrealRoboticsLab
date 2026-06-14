# Debug Visualization

Runtime overlays for inspecting the MuJoCo simulation during Play-in-Editor. Toggle them with number-row hotkeys, and tune their properties live from the Details panel on the Manager actor.

All overlays live on `UMjDebugVisualizer`, which is auto-created on the Manager. Input is handled by `UMjInputHandler`.

![Joint axes, contact arrows, and collision wireframes drawn over a robot during PIE](../images/placeholder.svg)

## Hotkeys

These are active during PIE:

| Key | Toggle | Purpose |
|-----|--------|---------|
| `1` | Contact force arrows | Yellow arrows at each active contact, scaled by normal force. |
| `2` | Articulation visual-mesh visibility | Hide imported visual meshes, keeping collision and debug overlays. |
| `3` | Articulation collision wireframes | Magenta wireframes of every articulation collision geom. |
| `4` | Joint axes and limit arcs | Coloured arrows showing each joint's axis and current/limit angle. |
| `5` | Quick-Convert collision wireframes | Same as `3` but for props spawned via Quick Convert. |
| `6` | Cycle body-shader overlay | Off, Island, Instance Segmentation, Semantic Segmentation, Off. See below. |
| `7` | Tendon/muscle tubes | Smooth tubes along every tendon, coloured by muscle activation. See below. |
| `P` | Pause/resume the physics thread | |
| `R` | Reset simulation state | |
| `O` | Toggle orbit and keyframe cameras | |
| `F` | Fire all impulse launchers | |

!!! tip
    Press `2` to hide visual meshes and `3` to show collision wireframes when you want to inspect contact shapes without visual clutter. See [Geometry & Collision](geometry.md) for what the collision overlay draws.

## Body-shader overlay (key `6`)

This cycles through three modes plus Off, repainting every articulation geom and Quick-Convert mesh. Original materials are cached on first use and restored when the overlay returns to Off or PIE ends.

- **Island** colours each body by its constraint island, the group of bodies connected through active contacts, joint limits, equality constraints, or friction loss. The colouring mirrors MuJoCo's native visualiser. Bodies with no degrees of freedom (worldbody, welded-to-world) read neutral grey.
- **Instance Segmentation** gives each body a unique hue, useful for per-body masks.
- **Semantic Segmentation** shares a hue across all instances of the same articulation class, so two Blueprints from the same class read as one colour.

When **sleep modulation** is on (default), sleeping bodies are dimmed and desaturated. Tune the value and saturation scales live in the Details panel.

!!! warning
    This overlay swaps real materials, so anything that renders the level (scene captures, `UMjCamera`, screenshot tools) captures the overlay rather than the underlying scene. It is a viewport debug tool. For persistent, clean RGB, depth, or mask streams, use per-camera capture modes instead, which are orthogonal to this overlay. See [Sensors & Cameras](sensors_cameras.md).

## Tendon and muscle tubes (key `7`)

Renders every tendon as a smooth tube. For muscle actuators, both the tube radius and colour track the muscle's activation in real time (relaxed reads thin and dark blue, contracted reads thick and bright red). Limited tendons without a muscle driver colour by their normalised length; neutral tendons read mid-purple. The path follows MuJoCo's post-kinematics tendon route, curving around any wrapping geoms. Tune `TendonTubeRadius` and `TendonArcSubdivisions` in the Details panel.

## Next steps

- [Geometry & Collision](geometry.md) for contact filtering and what the collision overlay shows.
- [Interaction](interaction.md) for mouse perturbation, which uses these overlays for its drag gizmos.
- [Simulate Dashboard](dashboard.md) for the in-editor control panel.
