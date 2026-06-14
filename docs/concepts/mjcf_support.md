# MJCF Support

What URLab parses from MJCF and writes back to a MuJoCo spec. This is a reference for which elements and attributes round-trip, so you can tell ahead of time whether a model will import cleanly.

Checked against MuJoCo upstream `main` (header version `3010000`, that is 3.10.0-dev). Status terms:

- **Supported** parses on import and exports back to the spec.
- **Missing** is not parsed or written; MuJoCo's own default applies if the model relies on it.

URLab models MJCF through `UMjComponent` subclasses. Most attribute-to-field mapping is generated from the MuJoCo schema; see [Codegen](../contributing/codegen.md) for how coverage is kept in sync with each MuJoCo version. The import flow itself is in the [Importing guide](../guides/importing.md).

## Coverage at a glance

| Category | Supported | Missing | Total |
|---|---:|---:|---:|
| body | 7 | 3 | 10 |
| joint | 18 | 2 | 20 |
| geom | 22 | 5 | 27 |
| site | 10 | 2 | 12 |
| actuator (common) | 14 | 2 | 16 |
| actuator types | 10 | 0 | 10 |
| sensor types | 41 | 0 | 41 |
| tendon | 16 | 1 | 17 |
| equality | 10 | 0 | 10 |
| default | 6 | 5 | 11 |
| compiler | 5 | 2 | 7 |
| option | 17 | 3 | 20 |
| keyframe | 6 | 0 | 6 |
| contact | 2 | 0 | 2 |
| asset | 3 | 1 | 4 |
| flexcomp | 22 | 2 | 24 |

The per-element notes below list only the attributes worth flagging (the gaps and the import-specific behaviour). Anything not called out as Missing is supported.

## body

Supported: `name`, `childclass`, `pos`, `quat`, `axisangle`, `xyaxes`, `zaxis`, `euler`, `mocap`, `gravcomp`, `sleep`. All orientation forms resolve through `OrientationToMjQuat`, honouring the `<compiler>` angle and eulerseq settings. `mocap` maps to `bDrivenByUnreal` so Unreal can drive the body kinematically.

Missing: `user`.

## joint

Supported includes `type`, `axis`, `range`, `springdamper`, `stiffness` and `damping` (up to three polynomial coefficients each), `armature`, `frictionloss`, `actuatorfrcrange`, and the full `sol*limit` / `sol*friction` set. Slide-joint `range` and `ref` auto-convert metres and centimetres.

Missing: `actuatorgravcomp`, `user`.

## geom

Supported includes `type`, `size`, `fromto`, `contype`, `conaffinity`, `condim`, `priority`, `friction`, `mass`, `density`, `material`, `rgba`, `mesh`, all orientation forms, and the `sol*` pair. Type maps to a UE visualizer where one exists: box, sphere, cylinder, and capsule (an engine cylinder plus two sphere caps). Ellipsoid, plane, hfield, and sdf use the base `UMjGeom` with no visualizer mesh.

Missing: `hfield` (the geom type is recognised but the hfield asset is not imported from `<asset>`), `fitscale`, `shellinertia`, `solmix` (bound in the runtime view but not imported / exported via XML), `fluidshape`, `fluidcoef`, `user`.

!!! note "fromto resolution"
    `fromto` on geoms and sites is decomposed at import into `pos`, `quat`, and the half-length size slot, leaving the radius to come from the default chain. See the [Geometry guide](../guides/geometry.md).

## site

Supported: `name`, `class`, `type`, `group`, `pos`, all orientation forms, `size`, `fromto`, `rgba`.

Missing: `material`, `user`.

## actuator

Common attributes supported: `class`, `group`, `ctrllimited`, `forcelimited`, `ctrlrange`, `forcerange`, `lengthrange`, `gear`, `cranklength`, the four transmission targets (`joint` / `tendon` / `site` / `body`), `actearly`, `actlimited`, `actrange`.

Missing common: `damping` (not on the base actuator, though the `damper` subtype's `kv` covers that case), `armature`, `user`.

All ten actuator types are supported: `general`, `motor`, `position`, `velocity`, `intvelocity`, `damper`, `cylinder`, `muscle`, `adhesion`, `dcmotor`. Preset types use the matching `mjs_setTo*` function; `general` writes raw `gainprm` / `biasprm` / `dynprm`.

!!! note "biastype / gaintype / dyntype"
    `<general>` actuators round-trip their `biastype`, `gaintype`, and `dyntype` so affine PD loops survive import. See the [Controllers guide](../guides/controllers.md).

## sensor

All 41 sensor types are supported, imported via a tag-name map and exported through `mjsSensor->type`. This spans touch / IMU sensors, joint / tendon / actuator readouts, frame and subtree quantities, geometric queries (`geomdist`, `geomnormal`, `geomfromto`), `contact`, energy, `clock`, `tactile`, `plugin`, and `user`.

Common attributes supported: `noise`, `cutoff`, target resolution (`objname` / `site` / `joint`), `refname`, `objtype`, `reftype`, `dim`, `class`. Missing: `user`.

## tendon

Spatial and fixed tendons are supported, including `stiffness` and `damping` (three-coefficient polynomials), `springlength`, `armature`, `range`, `actfrcrange`, the `sol*` set, `width`, and `rgba`. All four wrap types work: `joint`, `site`, `geom` (with sidesite), `pulley`.

Missing: `user`.

## equality

All types supported: `connect`, `weld`, `joint`, `tendon`, plus the flex constraints `flex`, `flexvert`, and `flexstrain`. Common attributes `active`, `solref`, `solimp` round-trip.

## default

The `<default>` class hierarchy (including nesting) is supported, and these children export: `geom`, `joint`, `site`, `camera`, `tendon`, and all actuator subtypes (polymorphic).

Missing from default export: `mesh`, `material`, `pair`, `equality`, `light`.

## compiler

Supported: `angle`, `eulerseq`, `meshdir`, `texturedir`, `assetdir`, `autolimits`. Missing: `coordinate` (always treated as `local`, the MuJoCo 3.x default), `settotalmass`.

## option

Supported: `timestep`, `gravity`, `wind`, `magnetic`, `density`, `viscosity`, `impratio`, `tolerance`, `iterations`, `ls_iterations`, `integrator`, `cone`, `solver`, the `noslip_*` and `ccd_*` pairs, and the `MultiCCD` / `Sleep` enable flags.

Missing: `o_margin`, `o_solref`, `o_solimp`, the `mpr_*` and `sdf_*` solver tuning groups.

## keyframe, contact, asset

- **keyframe**: `name`, `time`, `qpos` (auto-padded for free joints), `qvel`, `act`, `ctrl`, `mpos`, `mquat` all round-trip.
- **contact**: `pair` and `exclude` both import and export fully.
- **asset**: `mesh`, `material`, and `texture` import (GLB / STL / OBJ meshes; PNG / JPG / BMP / TGA textures applied to material instances). Missing: `hfield` is not imported from `<asset>`.

## flexcomp

`<flexcomp>` is parsed into a `UMjFlexcomp` and, at registration, serialised back to an MJCF fragment that MuJoCo's own parser expands via `mjs_attach`. All geometry types (grid / box / cylinder / ellipsoid / square / disc / circle / mesh / direct) and DOF modes (full / radial / trilinear / quadratic) work without a plugin-side reimplementation. Sub-elements `<contact>`, `<edge>`, `<elasticity>`, and `<pin>` are supported through `bOverride_*` rules.

Missing: the `pingrid` and `pinrange` pin selectors (only `id` and `gridrange` are supported).

The compiled `<flex>` element is read back at runtime for visualization; `UMjFlexcomp::Bind` caches `flex_vertadr`, `flex_vertnum`, and the triangle index list and updates from `mjData.flexvert_xpos` each tick.
