"""Manifest for MuJoCo 4 mjSpec -> UE 5.7 UENUM / USTRUCT / Marshal codegen."""

TARGET_ENUMS = [
    "mjtGeom",
    "mjtJoint",
    "mjtActuator",
    "mjtDyn",
    "mjtGain",
    "mjtBias",
    "mjtTrn",
    "mjtSensor",
    "mjtObj",
    "mjtDataType",
    "mjtStage",
    "mjtCamLight",
    "mjtProjection",
    "mjtLightType",
    "mjtTexture",
    "mjtTextureRole",
    "mjtColorSpace",
    "mjtBuiltin",
    "mjtMark",
    "mjtMeshInertia",
    "mjtGeomInertia",
    "mjtEmbed",
    "mjtInertiaFromGeom",
    "mjtLimited",
    "mjtWrap",
    "mjtEq",
    "mjtAlignFree",
    "mjtSleepPolicy",
    "mjtScaling",
    "mjtOrientation",
    "mjtSameFrame",
    "mjtCone",
    "mjtJacobian",
    "mjtSolver",
    "mjtIntegrator",
]

# Structs to emit FMJ<Name>Spec USTRUCTs for.
# Order matters if any struct references another, though ours are flat value structs.
TARGET_STRUCTS = [
    "mjsBody",
    "mjsFrame",
    "mjsGeom",
    "mjsJoint",
    "mjsSite",
    "mjsCamera",
    "mjsLight",
    "mjsActuator",
    "mjsSensor",
    "mjsTendon",
    "mjsTexture",
    "mjsMaterial",
    "mjsMesh",
]

# Fields skipped from FMJ*Spec (owned by USceneComponent transform, UObject name,
# internal MuJoCo pointers, or handled via dedicated UE component properties).
SKIP_FIELDS = {
    "*": {
        "element",
        "info",
        "userdata",
        "plugin",
        "authored",
    },
    "mjsBody": {"pos", "quat", "alt", "plugin", "explicitinertial"},
    "mjsFrame": {"pos", "quat", "alt"},
    "mjsGeom": {"pos", "quat", "alt", "plugin", "hfieldname", "meshname", "material", "fitscale"},
    "mjsJoint": {"pos"},
    "mjsSite": {"pos", "quat", "alt", "material", "meshname"},
    "mjsCamera": {"pos", "quat", "alt", "targetbody"},
    "mjsLight": {"pos", "dir", "targetbody", "texture"},
    "mjsActuator": {"target", "slidersite", "cranksite"},
    "mjsSensor": {"objname", "refname", "plugin"},
    "mjsTendon": {"material"},
    "mjsTexture": {"data", "ndata", "content_type", "file", "cubefiles"},
    "mjsMaterial": {"textures"},
    "mjsMesh": {
        "content_type",
        "file",
        "plugin",
        "uservert",
        "usernormal",
        "usertexcoord",
        "userface",
        "userfacenormal",
        "userfacetexcoord",
        "usertet",
    },
}

# Explicit unit/coordinate overrides for fields where mjcf.schema dim needs
# vector/coordinate-frame awareness or where introspect field != schema attr.
# Values:
#   "len"       : scalar or array of lengths (UE cm <-> MJ m)
#   "vec3_pos"  : 3D position/offset vector (UE FVector cm LHS <-> MJ double[3] m RHS)
#   "vec3_dir"  : 3D unit/axis direction (UE FVector LHS <-> MJ double[3] RHS)
#   "quat"      : quaternion (UE FQuat LHS <-> MJ double[4] wxyz RHS)
#   "fromto"    : 6-element two-point segment (UE 2x FVector cm LHS <-> MJ double[6] m RHS)
#   "angle"     : scalar or array of angles (UE deg <-> MJ rad; note mjSpec stores rad when degree=false)
#   "bool"      : mjtBool <-> bool
#   "color"     : float[4] / double[3] RGBA/RGB
FIELD_UNIT_OVERRIDES = {
    "mjsBody.ipos": "vec3_pos",
    "mjsBody.iquat": "quat",
    "mjsGeom.fromto": "fromto",
    "mjsGeom.size": "len",
    "mjsGeom.margin": "len",
    "mjsGeom.gap": "len",
    "mjsGeom.fluidcoef": "raw",
    "mjsJoint.axis": "vec3_dir",
    "mjsJoint.margin": "len",
    "mjsSite.fromto": "fromto",
    "mjsSite.size": "len",
    "mjsMesh.refpos": "vec3_pos",
    "mjsMesh.refquat": "quat",
    "mjsMesh.scale": "vec3_scale",
    "mjsFrame.scale": "vec3_scale",
    "mjsLight.range": "len",
    "mjsLight.bulbradius": "len",
    "mjsLight.cutoff": "angle",
    "mjsCamera.fovy": "angle",
    "mjsCamera.ipd": "len",
    "mjsCamera.sensor_size": "len",
    "mjsCamera.focal_length": "len",
    "mjsTendon.springlength": "len",
    "mjsTendon.margin": "len",
    "mjsTendon.width": "len",
}

# Actuator shortcut parameter mapping for UMJActuatorComponent
ACTUATOR_SHORTCUTS = [
    ("General", "mjACTUATOR_GENERAL"),
    ("Motor", "mjACTUATOR_MOTOR"),
    ("Position", "mjACTUATOR_POSITION"),
    ("Velocity", "mjACTUATOR_VELOCITY"),
    ("IntVelocity", "mjACTUATOR_INTVELOCITY"),
    ("Damper", "mjACTUATOR_DAMPER"),
    ("Cylinder", "mjACTUATOR_CYLINDER"),
    ("Muscle", "mjACTUATOR_MUSCLE"),
    ("Adhesion", "mjACTUATOR_ADHESION"),
    ("PID", "mjACTUATOR_PID"),
    ("Orientation", "mjACTUATOR_ORIENTATION"),
    ("DCMotor", "mjACTUATOR_DCMOTOR"),
]
