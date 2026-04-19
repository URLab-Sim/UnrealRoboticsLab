# MJCF Schema Coverage

Checked against: **MuJoCo 3.7.0** (C API / mjSpec)
Plugin version: UnrealRoboticsLab main branch, 2026-04-18

## Summary

| Category | Supported | Import-Only | Missing | Total |
|----------|-----------|-------------|---------|-------|
| body | 7 | 0 | 3 | 10 |
| joint | 18 | 0 | 2 | 20 |
| geom | 22 | 0 | 5 | 27 |
| site | 10 | 0 | 2 | 12 |
| actuator (common) | 14 | 0 | 2 | 16 |
| actuator types | 10 | 0 | 0 | 10 |
| sensor types | 41 | 0 | 0 | 41 |
| tendon | 16 | 0 | 1 | 17 |
| equality | 10 | 0 | 0 | 10 |
| default | 6 | 0 | 5 | 11 |
| compiler | 5 | 0 | 2 | 7 |
| option | 17 | 0 | 3 | 20 |
| keyframe | 6 | 0 | 0 | 6 |
| contact | 2 | 0 | 0 | 2 |
| asset | 3 | 0 | 1 | 4 |

---

## body

| Attribute | Status | Notes |
|-----------|--------|-------|
| name | SUPPORTED | Import: `ReadAttrString("name")`. Export: `mjs_setName` via `Setup()` |
| childclass | SUPPORTED | Import: `ReadAttrString("childclass")`. Export: `mjs_setString(BodyToAttachTo->childclass)` |
| pos | SUPPORTED | Import: `ParseVector(pos)` + coord convert. Export: `UEToMjPosition` in `Setup()` |
| quat | SUPPORTED | Import: via `OrientationToMjQuat`. Export: via `Setup()` transform |
| axisangle | SUPPORTED | Import: via `OrientationToMjQuat` (priority 2) |
| xyaxes | SUPPORTED | Import: via `OrientationToMjQuat` (priority 3) |
| zaxis | SUPPORTED | Import: via `OrientationToMjQuat` (priority 4) |
| euler | SUPPORTED | Import: via `OrientationToMjQuat` (priority 5, respects eulerseq) |
| mocap | SUPPORTED | Import: `ReadAttrBool("mocap")` -> `bDrivenByUnreal`. Export: `BodyToAttachTo->mocap = 1` |
| gravcomp | SUPPORTED | Import: `ReadAttrFloat("gravcomp")`. Export: `BodyToAttachTo->gravcomp = Gravcomp` |
| sleep | SUPPORTED | Import: `ReadAttrString("sleep")` -> enum. Export: `BodyToAttachTo->sleep` |
| user | MISSING | Not parsed or exported |

## joint

| Attribute | Status | Notes |
|-----------|--------|-------|
| name | SUPPORTED | Import: via MjComponent base. Export: `mjs_setName` in `RegisterToSpec` |
| class | SUPPORTED | Import: `ReadAttrString("class")`. Export: `ResolveDefault` |
| type | SUPPORTED | Import: parsed string -> enum. Export: `Joint->type` |
| group | SUPPORTED | Import: `ReadAttrInt("group")`. Export: `Joint->group` |
| pos | SUPPORTED | Import: `ReadAttrString("pos")` + coord convert. Export: `UEToMjPosition` |
| axis | SUPPORTED | Import: `ReadAttrString("axis")` + Y negate. Export: writes `Joint->axis` |
| springdamper | SUPPORTED | Import: `ReadAttrFloatArray("springdamper")` -> Stiffness + Damping |
| limited | SUPPORTED | Import: `ReadAttrBool("limited")`. Export: `Joint->limited` |
| solreflimit | SUPPORTED | Import + Export via `Joint->solref_limit` |
| solimplimit | SUPPORTED | Import + Export via `Joint->solimp_limit` |
| solreffriction | SUPPORTED | Import + Export via `Joint->solref_friction` |
| solimpfriction | SUPPORTED | Import + Export via `Joint->solimp_friction` |
| stiffness | SUPPORTED | Import: `ReadAttrFloatArray` (up to 3 polynomial coefficients). Export: `Joint->stiffness[0..2]` |
| range | SUPPORTED | Import + Export. Slide joints auto-convert m<->cm |
| actuatorfrcrange | SUPPORTED | Import: `ReadAttrFloatArray`. Export: `Joint->actfrcrange` |
| margin | SUPPORTED | Import: `ReadAttrFloat`. Export: `Joint->margin` |
| ref | SUPPORTED | Import: `ReadAttrFloat`. Export: `Joint->ref`. Slide: m<->cm |
| springref | SUPPORTED | Import: `ReadAttrFloat`. Export: `Joint->springref` |
| armature | SUPPORTED | Import: `ReadAttrFloat`. Export: `Joint->armature` |
| damping | SUPPORTED | Import: `ReadAttrFloatArray` (up to 3 polynomial coefficients). Export: `Joint->damping[0..2]` |
| frictionloss | SUPPORTED | Import: `ReadAttrFloat`. Export: `Joint->frictionloss` |
| actuatorgravcomp | MISSING | Not parsed |
| user | MISSING | Not parsed or exported |

## geom

| Attribute | Status | Notes |
|-----------|--------|-------|
| name | SUPPORTED | Import: `GetAttribute("name")`. Export: `SetSpecElementName` |
| class | SUPPORTED | Import: `ReadAttrString("class")`. Export: `ResolveDefault` |
| type | SUPPORTED | Import: string -> enum (plane/hfield/sphere/capsule/ellipsoid/cylinder/box/mesh/sdf). Export: writes `Geom->type`. Primitives with a dedicated UE visualizer class: box (`UMjBox`), sphere (`UMjSphere`), cylinder (`UMjCylinder`), capsule (`UMjCapsule`, composite of engine cylinder + two sphere caps). Other types (ellipsoid, plane, hfield, sdf) use the base `UMjGeom` without a visualizer mesh. |
| contype | SUPPORTED | Import: `ReadAttrInt`. Export: `Geom->contype` |
| conaffinity | SUPPORTED | Import: `ReadAttrInt`. Export: `Geom->conaffinity` |
| condim | SUPPORTED | Import: `ReadAttrInt`. Export: `Geom->condim` |
| group | SUPPORTED | Import: `ReadAttrInt`. Export: `Geom->group` |
| priority | SUPPORTED | Import: `ReadAttrInt`. Export: `Geom->priority` |
| size | SUPPORTED | Import: `ReadAttrFloatArray` (handles 1/2/3 params). Export: writes per `SizeParamsCount` |
| material | SUPPORTED | Import: `ReadAttrString("material")`. Export: material instance created at import |
| friction | SUPPORTED | Import: `ReadAttrFloatArray`. Export: `Geom->friction` |
| mass | SUPPORTED | Import: `ReadAttrFloat`. Export: `Geom->mass` |
| density | SUPPORTED | Import: `ReadAttrFloat`. Export: `Geom->density` |
| solref | SUPPORTED | Import: `ReadAttrFloatArray`. Export: `Geom->solref` |
| solimp | SUPPORTED | Import: `ReadAttrFloatArray`. Export: `Geom->solimp` |
| margin | SUPPORTED | Import: `ReadAttrFloat`. Export: `Geom->margin` |
| gap | SUPPORTED | Import: `ReadAttrFloat`. Export: `Geom->gap` |
| fromto | SUPPORTED | Import: resolves to pos/quat/size. Export: raw fromto or decomposed |
| pos | SUPPORTED | Import: `ParseVector` + coord convert. Export: `UEToMjPosition` |
| quat | SUPPORTED | Import: via `OrientationToMjQuat`. Export: `UEToMjRotation` |
| axisangle | SUPPORTED | Import: via `OrientationToMjQuat` |
| xyaxes | SUPPORTED | Import: via `OrientationToMjQuat` |
| zaxis | SUPPORTED | Import: via `OrientationToMjQuat` |
| euler | SUPPORTED | Import: via `OrientationToMjQuat` |
| mesh | SUPPORTED | Import: `ReadAttrString("mesh")`. Export: `mjs_setString(Geom->meshname)` |
| rgba | SUPPORTED | Import: `ReadAttrFloatArray`. Export: `Geom->rgba` |
| hfield | MISSING | Geom type recognized; hfield asset not imported from XML `<asset>` |
| fitscale | MISSING | Not parsed |
| shellinertia | MISSING | Not parsed |
| solmix | MISSING | Bound in runtime view (`geom_solmix`) but not imported/exported via XML |
| fluidshape | MISSING | Not parsed |
| fluidcoef | MISSING | Not parsed |
| user | MISSING | Not parsed |

## site

| Attribute | Status | Notes |
|-----------|--------|-------|
| name | SUPPORTED | Import: via MjComponent base. Export: `SetSpecElementName` |
| class | SUPPORTED | Import: via MjComponent base. Export: `ResolveDefault` |
| type | SUPPORTED | Import: string -> enum (sphere/capsule/ellipsoid/cylinder/box). Export: `site->type` |
| group | SUPPORTED | Import: `ReadAttrInt`. Export: `site->group` |
| pos | SUPPORTED | Import: parsed + coord convert. Export: `UEToMjPosition` |
| quat | SUPPORTED | Import: via `OrientationToMjQuat`. Export: `UEToMjRotation` |
| axisangle | SUPPORTED | Import: via `OrientationToMjQuat` |
| xyaxes | SUPPORTED | Import: via `OrientationToMjQuat` |
| zaxis | SUPPORTED | Import: via `OrientationToMjQuat` |
| euler | SUPPORTED | Import: via `OrientationToMjQuat` |
| size | SUPPORTED | Import: `ReadAttrFloatArray`. Export: `site->size` |
| fromto | SUPPORTED | Import: resolves to pos/quat/size. Export: raw fromto |
| rgba | SUPPORTED | Import: `ReadAttrFloatArray`. Export: `site->rgba` |
| material | MISSING | Not parsed |
| user | MISSING | Not parsed |

## actuator (common attributes)

| Attribute | Status | Notes |
|-----------|--------|-------|
| name | SUPPORTED | Export: `SetSpecElementName` in `RegisterToSpec` |
| class | SUPPORTED | Import: `ReadAttrString("class")`. Export: `ResolveDefault` |
| group | SUPPORTED | Import: `ReadAttrInt`. Export: `Actuator->group` |
| ctrllimited | SUPPORTED | Import: `ReadAttrBool`. Export: `Actuator->ctrllimited` |
| forcelimited | SUPPORTED | Import: `ReadAttrBool`. Export: `Actuator->forcelimited` |
| ctrlrange | SUPPORTED | Import: `ReadAttrFloatArray`. Export: `Actuator->ctrlrange` |
| forcerange | SUPPORTED | Import: `ReadAttrFloatArray`. Export: `Actuator->forcerange` |
| lengthrange | SUPPORTED | Import: `ReadAttrFloatArray`. Export: `Actuator->lengthrange` |
| gear | SUPPORTED | Import: `ReadAttrFloatArray`. Export: `Actuator->gear` |
| cranklength | SUPPORTED | Import: `ReadAttrFloat`. Export: `Actuator->cranklength` |
| joint | SUPPORTED | Import: sets `TransmissionType::Joint`. Export: via `trntype` + `target` |
| tendon | SUPPORTED | Import: sets `TransmissionType::Tendon`. Export: via `trntype` + `target` |
| site | SUPPORTED | Import: sets `TransmissionType::Site`. Export: via `trntype` + `target` |
| body | SUPPORTED | Import: sets `TransmissionType::Body`. Export: via `trntype` + `target` |
| actearly | SUPPORTED | Import: `ReadAttrBool`. Export: `Actuator->actearly` |
| actlimited | SUPPORTED | Import: `ReadAttrBool`. Export: `Actuator->actlimited` |
| actrange | SUPPORTED | Import: `ReadAttrFloatArray`. Export: `Actuator->actrange` |
| damping | MISSING | Not parsed in base actuator (but subtype `kv` covers damper case) |
| armature | MISSING | Not parsed in base actuator |
| user | MISSING | Not parsed |

## actuator types

| Type | Status | Notes |
|------|--------|-------|
| general | SUPPORTED | `MjGeneralActuator`. Exports raw gainprm/biasprm/dynprm directly |
| motor | SUPPORTED | `MjMotorActuator`. Uses `mjs_setToMotor` |
| position | SUPPORTED | `MjPositionActuator`. Parses `kp`. Uses `mjs_setToPosition` |
| velocity | SUPPORTED | `MjVelocityActuator`. Parses `kv`. Uses `mjs_setToVelocity` |
| intvelocity | SUPPORTED | `MjIntVelocityActuator`. Parses `kp`. Uses `mjs_setToIntvelocity` |
| damper | SUPPORTED | `MjDamperActuator`. Parses `kv`. Uses `mjs_setToDamper` |
| cylinder | SUPPORTED | `MjCylinderActuator`. Parses `timeconst/bias/area/diameter`. Uses `mjs_setToCylinder` |
| muscle | SUPPORTED | `MjMuscleActuator`. Uses `mjs_setToMuscle` |
| adhesion | SUPPORTED | `MjAdhesionActuator`. Parses `gain`. Uses `mjs_setToAdhesion` |
| dcmotor | SUPPORTED | `MjDcMotorActuator`. Parses `motorconst`, `resistance`, `nominal`, `saturation`, `inductance`, `cogging`, `controller`, `thermal`, `lugre`, `input`. Uses `mjs_setToDCMotor` (MuJoCo 3.7.0+). |

## sensor types

All 41 sensor types are SUPPORTED (import via tag-name map, export via `mjsSensor->type`):

| Type | Status |
|------|--------|
| touch | SUPPORTED |
| accelerometer | SUPPORTED |
| velocimeter | SUPPORTED |
| gyro | SUPPORTED |
| force | SUPPORTED |
| torque | SUPPORTED |
| magnetometer | SUPPORTED |
| camprojection | SUPPORTED |
| rangefinder | SUPPORTED |
| jointpos | SUPPORTED |
| jointvel | SUPPORTED |
| jointactfrc | SUPPORTED |
| tendonpos | SUPPORTED |
| tendonvel | SUPPORTED |
| tendonactfrc | SUPPORTED |
| actuatorpos | SUPPORTED |
| actuatorvel | SUPPORTED |
| actuatorfrc | SUPPORTED |
| ballquat | SUPPORTED |
| ballangvel | SUPPORTED |
| jointlimitpos | SUPPORTED |
| jointlimitvel | SUPPORTED |
| jointlimitfrc | SUPPORTED |
| tendonlimitpos | SUPPORTED |
| tendonlimitvel | SUPPORTED |
| tendonlimitfrc | SUPPORTED |
| framepos | SUPPORTED |
| framequat | SUPPORTED |
| framexaxis | SUPPORTED |
| frameyaxis | SUPPORTED |
| framezaxis | SUPPORTED |
| framelinvel | SUPPORTED |
| frameangvel | SUPPORTED |
| framelinacc | SUPPORTED |
| frameangacc | SUPPORTED |
| subtreecom | SUPPORTED |
| subtreelinvel | SUPPORTED |
| subtreeangmom | SUPPORTED |
| insidesite | SUPPORTED |
| geomdist | SUPPORTED |
| geomnormal | SUPPORTED |
| geomfromto | SUPPORTED |
| contact | SUPPORTED |
| e_potential | SUPPORTED |
| e_kinetic | SUPPORTED |
| clock | SUPPORTED |
| tactile | SUPPORTED |
| plugin | SUPPORTED |
| user | SUPPORTED |

Sensor common attributes:

| Attribute | Status | Notes |
|-----------|--------|-------|
| name | SUPPORTED | Via `SetSpecElementName` |
| noise | SUPPORTED | Import: `ReadAttrFloat`. Export: `Sensor->noise` |
| cutoff | SUPPORTED | Import: `ReadAttrFloat`. Export: `Sensor->cutoff` |
| objname/site/joint/etc | SUPPORTED | Import: tries each target attr. Export: `Sensor->objname` |
| refname | SUPPORTED | Import: `GetAttribute("refname")`. Export: `Sensor->refname` |
| objtype | SUPPORTED | Import: string -> enum. Export: `Sensor->objtype` |
| reftype | SUPPORTED | Import: string -> enum. Export: `Sensor->reftype` |
| dim | SUPPORTED | Import: parsed from attr. Export: `Sensor->dim` |
| class | SUPPORTED | Import: `ReadAttrString("class")` |
| user | MISSING | Not parsed |

## tendon

| Attribute | Status | Notes |
|-----------|--------|-------|
| name | SUPPORTED | Import: via MjComponent. Export: `mjs_setName` |
| class | SUPPORTED | Import: `ReadAttrString("class")`. Export: `ResolveDefault` |
| stiffness | SUPPORTED | Import: `ReadAttrFloatArray` (up to 3 polynomial coefficients). Export: `Tendon->stiffness[0..2]` |
| springlength | SUPPORTED | Import + Export |
| damping | SUPPORTED | Import: `ReadAttrFloatArray` (up to 3 polynomial coefficients). Export: `Tendon->damping[0..2]` |
| frictionloss | SUPPORTED | Import + Export |
| armature | SUPPORTED | Import + Export |
| limited | SUPPORTED | Import + Export |
| range | SUPPORTED | Import + Export |
| margin | SUPPORTED | Import + Export |
| actfrclimited | SUPPORTED | Import + Export |
| actfrcrange | SUPPORTED | Import + Export |
| solreflimit | SUPPORTED | Import + Export |
| solimplimit | SUPPORTED | Import + Export |
| solreffriction | SUPPORTED | Import + Export |
| solimpfriction | SUPPORTED | Import + Export |
| width | SUPPORTED | Import + Export |
| rgba | SUPPORTED | Import + Export |
| group | SUPPORTED | Import + Export |
| user | MISSING | Not parsed |

Tendon wrap types:

| Wrap Type | Status | Notes |
|-----------|--------|-------|
| joint | SUPPORTED | `mjs_wrapJoint` with coef |
| site | SUPPORTED | `mjs_wrapSite` |
| geom | SUPPORTED | `mjs_wrapGeom` with sidesite |
| pulley | SUPPORTED | `mjs_wrapPulley` with divisor |

## equality

| Type | Status | Notes |
|------|--------|-------|
| connect | SUPPORTED | Parses body1/body2, anchor. Exports via `Eq->data[0..2]` |
| weld | SUPPORTED | Parses body1/body2, relpos, relquat, torquescale. Exports via `Eq->data[0..7]` |
| joint | SUPPORTED | Parses joint1/joint2, polycoef. Exports via `Eq->data` |
| tendon | SUPPORTED | Parses tendon1/tendon2, polycoef. Exports via `Eq->data` |
| flex | SUPPORTED | Parses `flex` attribute into `Obj1`. Fixes all edge lengths of a flex (MuJoCo 3.5.0+). |
| flexvert | SUPPORTED | Parses `flex` attribute into `Obj1`. Fixes all vertex lengths of a flex (MuJoCo 3.5.0+). |
| flexstrain | SUPPORTED | Parses `flex` attribute into `Obj1`. Constrains strain of a trilinear/quadratic flex (MuJoCo 3.6.0+). |

Equality common attributes:

| Attribute | Status | Notes |
|-----------|--------|-------|
| active | SUPPORTED | Import: `ReadAttrBool`. Export: `Eq->active` |
| solref | SUPPORTED | Import + Export |
| solimp | SUPPORTED | Import + Export |

## default

| Child Element | Status | Notes |
|---------------|--------|-------|
| class (attr) | SUPPORTED | Import: `GetAttribute("class")`. Nested defaults supported |
| geom | SUPPORTED | `GeomComp->ExportTo(def->geom)` |
| joint | SUPPORTED | `JointComp->ExportTo(def->joint)` |
| site | SUPPORTED | `SiteComp->ExportTo(def->site)` |
| camera | SUPPORTED | `CameraComp->ExportTo(def->camera)` |
| tendon | SUPPORTED | `TendonComp->ExportTo(def->tendon)` |
| general/motor/position/velocity/etc | SUPPORTED | `ActuatorComp->ExportTo(def->actuator)` (polymorphic) |
| mesh | MISSING | Not handled in default export |
| material | MISSING | Not handled in default export |
| pair | MISSING | Not handled in default export |
| equality | MISSING | Not handled in default export |
| light | MISSING | Not handled in default export |

## compiler

| Attribute | Status | Notes |
|-----------|--------|-------|
| angle | SUPPORTED | `ParseCompilerSettings` -> `bAngleInDegrees`. Used by `OrientationToMjQuat` |
| eulerseq | SUPPORTED | `ParseCompilerSettings` -> `EulerSeq`. Used for euler decomposition |
| meshdir | SUPPORTED | Parsed from `<compiler>`, used for mesh path resolution |
| texturedir | SUPPORTED | Parsed from `<compiler>`, used for texture path resolution |
| assetdir | SUPPORTED | Parsed, overrides meshdir for asset lookups |
| autolimits | SUPPORTED | Parsed into `bAutoLimits` (deferred to MuJoCo spec compiler) |
| coordinate | MISSING | Always assumes "local" (MuJoCo 3.x default) |
| settotalmass | MISSING | Not parsed |

## option

| Attribute | Status | Notes |
|-----------|--------|-------|
| timestep | SUPPORTED | Import from XML + `ApplyToSpec`. Runtime override supported |
| gravity | SUPPORTED | Import + coord convert (cm/s2, Y negate) |
| wind | SUPPORTED | Import + coord convert |
| magnetic | SUPPORTED | Import + coord convert |
| density | SUPPORTED | Import + export |
| viscosity | SUPPORTED | Import + export |
| impratio | SUPPORTED | Import + export |
| tolerance | SUPPORTED | Import + export |
| iterations | SUPPORTED | Import + export |
| ls_iterations | SUPPORTED | Import + export |
| integrator | SUPPORTED | Import: string -> enum (Euler/RK4/implicit/implicitfast) |
| cone | SUPPORTED | Import: string -> enum (pyramidal/elliptic) |
| solver | SUPPORTED | Import: string -> enum (PGS/CG/Newton) |
| noslip_iterations | SUPPORTED | Import + export (with override) |
| noslip_tolerance | SUPPORTED | Import + export (with override) |
| ccd_iterations | SUPPORTED | Import + export (with override) |
| ccd_tolerance | SUPPORTED | Import + export (with override) |
| flag (enableflags) | SUPPORTED | MultiCCD and Sleep flags handled |
| o_margin | MISSING | Not parsed |
| o_solref | MISSING | Not parsed |
| o_solimp | MISSING | Not parsed |
| mpr_iterations | MISSING | Not parsed |
| mpr_tolerance | MISSING | Not parsed |
| sdf_iterations | MISSING | Not parsed |
| sdf_initpoints | MISSING | Not parsed |

## keyframe/key

| Attribute | Status | Notes |
|-----------|--------|-------|
| name | SUPPORTED | Import: via MjComponent. Export: `mjs_setName` |
| time | SUPPORTED | Import: `ReadAttrFloat`. Export: `Key->time` |
| qpos | SUPPORTED | Import + Export. Auto-pads for free joints |
| qvel | SUPPORTED | Import + Export |
| act | SUPPORTED | Import + Export |
| ctrl | SUPPORTED | Import + Export |
| mpos | SUPPORTED | Import + Export |
| mquat | SUPPORTED | Import + Export |

## contact

| Element | Status | Notes |
|---------|--------|-------|
| pair | SUPPORTED | Parses geom1/geom2/condim/friction/solref/solimp/gap/margin. Full import+export |
| exclude | SUPPORTED | Parses body1/body2/name. Full import+export |

## asset

| Element | Status | Notes |
|---------|--------|-------|
| mesh | SUPPORTED | Parsed recursively via `ParseAssetsRecursive`. GLB/STL/OBJ imported as UE StaticMesh |
| material | SUPPORTED | Parsed into `FMuJoCoMaterialData`. Creates UE MaterialInstance with texture params |
| texture | SUPPORTED | Imported from disk (PNG/JPG/BMP/TGA). Applied to material instances |
| hfield | MISSING | Not parsed from `<asset>`. Geom type=hfield recognized but asset data not imported |
