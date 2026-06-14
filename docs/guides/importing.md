# Importing Robots

Drag a MuJoCo XML (MJCF) file into the Content Browser and URLab rebuilds the full model as an Unreal Blueprint, ready to simulate.

Grab an `.xml` from the [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie) or use your own, then drop it straight into Unreal.

![Dragging an MJCF file into the Unreal Content Browser and the resulting articulation Blueprint](../images/placeholder.svg)

## Import a model

1. Drag your `.xml` file into the Unreal **Content Browser**.
2. On first import, the editor prompts you to install the Python packages the importer needs (`trimesh`, `numpy`, `scipy`). By default these install into UE's bundled Python, so no external Python is required. You can also point the dialog at your own interpreter (conda, venv). The chosen path is saved to `Config/LocalUnrealRoboticsLab.ini` (per-machine, not committed).
3. The importer runs `Scripts/clean_meshes.py` to prepare meshes: it parses the XML to find referenced meshes, resolves GLB stem conflicts (for example `link1.obj` and `link1.stl` both producing `link1.glb`), converts meshes to GLB while preserving UVs, and writes a `_ue.xml` with updated references. If you skip the Python setup, the raw XML is used and some meshes may not display.
4. The factory builds an `AMjArticulation` Blueprint from the prepared model in four passes: assets (meshes, textures, materials), defaults (the class hierarchy), the worldbody (bodies, geoms, joints, sites), and the rest (actuators, sensors, tendons, equalities, keyframes, contact pairs and excludes).
5. Place the Blueprint in your level. It is ready to simulate.

!!! tip
    You can also prepare meshes ahead of time by running `python Scripts/clean_meshes.py path/to/robot.xml` yourself, then dragging the resulting `_ue.xml` into Unreal.

## What the importer reads

The importer honours the MJCF `<compiler>` element, so `angle="degree"` vs `"radian"`, `autolimits`, and `eulerseq` are all interpreted the way MuJoCo would interpret them.

Geoms written with `fromto` are resolved to explicit position, orientation, and size at import time. The component stores the resolved transform; the original `fromto` is not kept.

Cameras are imported with their intrinsics resolved. Where the MJCF specifies `focal`, `focalpixel`, or `sensorsize` instead of a vertical field of view, the importer derives `fovy` from those values so the Unreal camera matches the MuJoCo camera. See [Sensors & Cameras](sensors_cameras.md) for what you can do with imported cameras.

For the full list of MJCF elements URLab understands, see [MJCF Support](../concepts/mjcf_support.md).

## How defaults are preserved

MJCF `<default>` blocks define properties that child elements inherit by class name. URLab imports these as live components, not static notes. When the Blueprint is compiled back into a MuJoCo spec, each component resolves its class against the default chain exactly as raw XML would.

This means:

- Editing a default component changes every component that references that class.
- Nested defaults keep their inheritance chain.
- Removing a default makes its dependents fall back to their own explicit values.

See [Articulations](articulations.md) for editing defaults in the Blueprint editor.

## Materials, textures, and meshes

The importer creates one shared Unreal material instance per MJCF `<material>` name, so every geom that references the same material shares a single instance. Textures from `<texture>` elements are applied to the matching material instance.

Meshes are located relative to the XML file and copied into a `/Meshes/` subfolder under the import target. The importer prefers GLB, then falls back to raw OBJ/STL. If a mesh cannot be loaded, the geom is still created as a collision primitive (without a visual mesh), a warning is logged, and the rest of the import succeeds.

## Debugging an import

If something does not look right after import, enable `bSaveDebugXml` on the Manager actor. After compilation URLab writes `scene_compiled.xml` and `scene_compiled.mjb` to `Saved/URLab/`. Diff the original MJCF against the compiled XML to spot missing elements, wrong values, or broken default inheritance. The compiled XML can also be opened in native MuJoCo for cross-checking.

!!! note
    The XML is only read at import time. After that, the Unreal components are the source of truth. You can rearrange components, add sensors or cameras, override properties, and save the result as a reusable Blueprint.

## Next steps

- [Articulations](articulations.md) covers editing the imported Blueprint and building one from scratch.
- [Geometry & Collision](geometry.md) covers collision shapes and contact filtering.
- [Controllers](controllers.md) covers how control targets become joint torques.
