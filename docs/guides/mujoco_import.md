# MJCF Import

Unreal Robotics Lab can import standard MuJoCo XML (MJCF) files and reconstruct the full model hierarchy as Unreal components. Grab an `.xml` from the MuJoCo Menagerie or your own projects and drag it straight into the Content Browser.

---

## Importing

1. Drag your `.xml` file into the Unreal **Content Browser**.
2. `UMujocoImportFactory` automatically runs `Scripts/clean_meshes.py` as a subprocess:
   - Parses the XML to find all referenced mesh assets
   - Detects GLB stem conflicts (e.g., `link1.obj` and `link1.stl` both producing `link1.glb`) and renames them
   - Converts meshes to GLB (preserving UVs, stripping embedded textures)
   - Produces a `_ue.xml` with updated mesh references
   - Graceful fallback: if Python or trimesh is missing, the raw XML is used instead
3. The factory creates an `AMjArticulation` Blueprint via four parsing passes:
   - **Pass 1:** Assets (meshes with scales, textures with file paths, materials with RGBA/texture refs)
   - **Pass 2:** Defaults (class hierarchy as `UMjDefault` components)
   - **Pass 3:** Worldbody (recursive body/geom/joint/site hierarchy)
   - **Pass 4:** Actuators, sensors, tendons, equalities, keyframes, contact pairs/excludes
4. Place the Blueprint in your level. It is ready to simulate.

---

## What Gets Created

| MJCF Element | Unreal Component |
|-------------|-----------------|
| `<body>` | `UMjBody` (recursive hierarchy preserved) |
| `<joint>` | `UMjHingeJoint`, `UMjSlideJoint`, `UMjBallJoint`, `UMjFreeJoint` |
| `<geom>` | `UMjGeom` / primitive subclass |
| `<site>` | `UMjSite` |
| `<actuator>` | `UMjMotorActuator`, `UMjPositionActuator`, etc. |
| `<sensor>` | `UMjTouchSensor`, `UMjGyro`, etc. (30+ types) |
| `<tendon>` | `UMjTendon` |
| `<equality>` | `UMjEquality` |
| `<keyframe>` | `UMjKeyframe` |
| `<default>` | `UMjDefault` |
| `<camera>` | `UMjCamera` |

---

## Compiler Settings

The importer respects the MJCF `<compiler>` element:

| Attribute | Effect |
|-----------|--------|
| `angle="degree"` / `"radian"` | How joint limits and positions are interpreted |
| `autolimits="true"` | Enables limits automatically when `range` is set |
| `eulerseq` | Euler rotation sequence for body orientations |

---

## How Defaults Work

`<default>` blocks in MuJoCo XML define shared properties that child elements inherit by class name. When imported, these become `UMjDefault` components in the articulation's component tree.

These are **not** just for reference — they are live components. The plugin uses them when converting the Blueprint *back* into MuJoCo's internal representation during compilation. Each component checks its assigned `ClassName` against the default chain, and MuJoCo applies the inheritance at spec compilation time, exactly as it would with raw XML.

This means:

- Editing a `UMjDefault` component affects all components that reference that default class.
- The default hierarchy (nested defaults) is preserved and functional.
- If you remove a default, components that relied on it will fall back to their own explicit property values.

---

## FromTo Resolution

Geoms defined with the `fromto` attribute are resolved to explicit `pos`, `quat`, and `size` at import time. The resolution is type-aware:

| Geom Type | Size Mapping |
|-----------|-------------|
| Capsule / Cylinder | `size[1]` = half-length along the segment |
| Box / Ellipsoid | `size[2]` = half-length along the segment |

After import the component stores the resolved transform — `fromto` is not preserved.

---

## Materials & Textures

The importer creates shared Unreal material instances keyed by XML `<material>` name. All geoms referencing the same material name share one instance (e.g., every geom with `material="white"` uses `MI_white`). Textures referenced by `<texture>` elements are applied to the corresponding material instance.

---

## Mesh Assets

The importer looks for mesh files relative to the XML file's directory. Found meshes are copied to a `/Meshes/` subfolder under the import target to avoid texture name collisions, then registered in MuJoCo's VFS during compilation.

**Format priority:** FBX > GLB > OBJ. The importer tries multiple paths via `ImportSingleMesh()`: GLB (via Interchange), then raw OBJ/STL (via FBX factory), then FBX fallback. If all fail, the geom is created without a visual mesh but still exists as a collision primitive. A warning is logged and compilation still succeeds.

**Auto mesh preparation:** The `clean_meshes_trimesh.py` script runs automatically during import (see [Importing](#importing) above). You can also run it manually:

```bash
python Scripts/clean_meshes.py path/to/robot.xml
```

This produces a `_ue.xml` file with updated mesh references that you can drag into Unreal instead of the original XML.

---

## Debugging Import Issues

Enable `bSaveDebugXml` on the `AAMjManager` to save `scene_compiled.xml` and `scene_compiled.mjb` to `Saved/URLab/` after compilation. Diff the original MJCF against the compiled XML to identify import/export mismatches (missing elements, wrong property values, broken default inheritance). The compiled XML can also be loaded into native MuJoCo for verification. See [Developer Tools](developer_tools.md) for more.

## After Import

The imported articulation is a normal Unreal actor. You can rearrange components, add sensors or cameras, override properties, or save it as a reusable Blueprint. The XML is only read during import — after that, the Unreal components are the source of truth.

For full details on the import pipeline internals, see [Architecture](../architecture.md#import-pipeline).
