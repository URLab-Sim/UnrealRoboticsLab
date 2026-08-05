"""Draft `schema/mujoco.spec` from the three bootstrap snapshots.

This script is the mechanical half of milestone 1's schema bootstrap. It reads

  * ``snapshots/mjcf_schema.json``   -- the MJCF element tree, per-element XML
    attribute lists, and the ``mjMap`` keyword tables (the coverage universe),
  * ``snapshots/mjspec_fields.json`` -- mjSpec C++ field types/arities/comments
    (the source of field types and doc comments),
  * ``snapshots/spec_defaults.json`` -- ``mjs_default*`` struct dumps
    (the source of real, non-sentinel default values),

and the *curation rules* encoded below as plain data (variant groups, mixin
assignments, ref targets, unit=angle fields, element renames, enum assignments,
variable-arity ranges). It emits a deterministic ``schema/mujoco.spec`` that the
ProtoSpec IDL parser accepts.

The output is generated-and-curated: re-running after a snapshot refresh
reproduces the draft, and every human judgment lives here as reviewable data
rather than in ~3000 hand-typed schema lines. After the drift gate (DR-12)
takes over, this script is retired.

Run: ``python tools/bootstrap/draft_schema.py`` (stdlib only), or with ``--check``
to verify the checked-in schema is up to date.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from collections import OrderedDict

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
SNAP = os.path.join(ROOT, "snapshots")
OUT = os.path.join(ROOT, "schema", "mujoco.spec")


# --------------------------------------------------------------------------- #
# Curation rules (the human-judgment decisions, made once, as data)           #
# --------------------------------------------------------------------------- #

# Enums to emit: IDL name -> snapshot keyword-map name. Only those actually
# assigned to a field (see ATTR_ENUM / CTYPE_ENUM) are written out.
ENUM_MAPS = {
    "Integrator": "integrator_map",
    "Cone": "cone_map",
    "JacobianType": "jac_map",
    "SolverType": "solver_map",
    "Coordinate": "coordinate_map",
    "AngleUnit": "angle_map",
    "InertiaFromGeom": "TFAuto_map",
    "TriState": "TFAuto_map",
    "GeomType": "geom_map",
    "JointType": "joint_map",
    "FluidShape": "fluid_map",
    "LightType": "lighttype_map",
    "CamLightMode": "camlight_map",
    "CameraProjection": "projection_map",
    "TextureType": "texture_map",
    "TextureBuiltin": "builtin_map",
    "TextureMark": "mark_map",
    "ColorSpace": "colorspace_map",
    "TexRole": "texrole_map",
    "MeshInertia": "meshinertia_map",
    "MeshBuiltin": "meshbuiltin_map",
    "GainType": "gain_map",
    "BiasType": "bias_map",
    "DynType": "dyn_map",
    "DcMotorInput": "dcmotorinput_map",
    "InterpType": "interp_map",
    "DataType": "datatype_map",
    "NeedStage": "stage_map",
    "CompositeType": "comp_map",
    "FlexcompType": "fcomp_map",
    "FlexDof": "fdof_map",
    "FlexSelfCollide": "flexself_map",
    "Elastic2D": "elastic2d_map",
    "FlexEquality": "flexeq_map",
    "ContactReduce": "reduce_map",
    "CameraOutput": "camout_map",
    "LRMode": "lrmode_map",
    "RayData": "raydata_map",
    "ContactData": "condata_map",
    "BodySleep": "bodysleep_map",
}

# Enum assignment. Keyed first by (IDL element name, xml attr), then by bare attr
# for the unambiguous cases. A value of None means "not an enum here".
ATTR_ENUM = {
    ("Option", "integrator"): "Integrator",
    ("Option", "cone"): "Cone",
    ("Option", "jacobian"): "JacobianType",
    ("Option", "solver"): "SolverType",
    ("Compiler", "coordinate"): "Coordinate",
    ("Compiler", "angle"): "AngleUnit",
    ("Compiler", "inertiafromgeom"): "InertiaFromGeom",
    ("Geom", "type"): "GeomType",
    ("Geom", "fluidshape"): "FluidShape",
    ("CompositeGeom", "type"): "GeomType",
    ("Site", "type"): "GeomType",
    ("Joint", "type"): "JointType",
    ("CompositeJoint", "type"): "JointType",
    ("Light", "type"): "LightType",
    ("Camera", "mode"): "CamLightMode",
    ("Light", "mode"): "CamLightMode",
    ("Camera", "projection"): "CameraProjection",
    ("Camera", "output"): "CameraOutput",
    ("Texture", "type"): "TextureType",
    ("Texture", "mark"): "TextureMark",
    ("Texture", "colorspace"): "ColorSpace",
    ("MaterialLayer", "role"): "TexRole",
    ("Mesh", "inertia"): "MeshInertia",
    ("Mesh", "builtin"): "MeshBuiltin",
    ("ActuatorGeneral", "gaintype"): "GainType",
    ("ActuatorGeneral", "biastype"): "BiasType",
    ("ActuatorGeneral", "dyntype"): "DynType",
    ("DcMotor", "input"): "DcMotorInput",
    ("Composite", "type"): "CompositeType",
    ("Flexcomp", "type"): "FlexcompType",
    ("Flexcomp", "dof"): "FlexDof",
    ("Flex", "dof"): "FlexDof",
    ("FlexcompContact", "selfcollide"): "FlexSelfCollide",
    ("FlexContact", "selfcollide"): "FlexSelfCollide",
    ("FlexcompElasticity", "elastic2d"): "Elastic2D",
    ("FlexElasticity", "elastic2d"): "Elastic2D",
    ("FlexcompEdge", "equality"): "FlexEquality",
    ("SensorUser", "datatype"): "DataType",
    ("SensorUser", "needstage"): "NeedStage",
    ("SensorContact", "reduce"): "ContactReduce",
    ("SensorContact", "data"): "ContactData",
    ("Rangefinder", "data"): "RayData",
    ("LengthRange", "mode"): "LRMode",
    ("Body", "sleep"): "BodySleep",
}
# Attributes MuJoCo reads with `MapValues` (space-separated keyword LISTS OR'd
# into an intprm/output bitmask, not scalar enums): each is typed `EnumName[]`.
# Sites in src/xml/xml_native_reader.cc: camera `output` (camout_map, ~2078),
# rangefinder `data` (raydata_map, ~4254), contact-sensor `data` (condata_map,
# ~4517). Keyed by (IDL element, xml attr); the enum itself is set in ATTR_ENUM.
ENUM_LIST_ATTRS = {
    ("Camera", "output"),
    ("Rangefinder", "data"),
    ("SensorContact", "data"),
}
# Tri-state limit flags (false/true/auto), unambiguous by attr name.
_TRISTATE_ATTRS = {
    "limited",
    "actuatorfrclimited",
    "ctrllimited",
    "forcelimited",
    "actlimited",
}
_GLOBAL_ENUM = {"interp": "InterpType"}

# Element name assignment, keyed by full MJCF tree path. Anything not listed is
# PascalCase(tag). These resolve tag collisions (a `joint` under body, equality,
# tendon/fixed, and composite are four different elements) and reuse a single
# element where identical MJCF nodes recur (config, the plugin-attachment stub,
# skin, bone, material layer, flex contact/elasticity).
PATH_NAMES = {
    "mujoco": "Model",
    "mujoco/compiler": "Compiler",
    "mujoco/compiler/lengthrange": "LengthRange",
    "mujoco/option": "Option",
    "mujoco/option/flag": "Flag",
    "mujoco/size": "Size",
    "mujoco/visual": "Visual",
    "mujoco/visual/global": "VisualGlobal",
    "mujoco/visual/quality": "VisualQuality",
    "mujoco/visual/headlight": "VisualHeadlight",
    "mujoco/visual/map": "VisualMap",
    "mujoco/visual/scale": "VisualScale",
    "mujoco/visual/rgba": "VisualRgba",
    "mujoco/statistic": "Statistic",
    "mujoco/default": "Default",
    "mujoco/default/equality": "EqualityDefault",
    "mujoco/default/tendon": "TendonDefault",
    "mujoco/extension": "Extension",
    "mujoco/extension/plugin": "PluginDef",
    "mujoco/extension/plugin/instance": "PluginInstance",
    "mujoco/extension/plugin/instance/config": "Config",
    "mujoco/custom": "Custom",
    "mujoco/custom/numeric": "Numeric",
    "mujoco/custom/text": "Text",
    "mujoco/custom/tuple": "Tuple",
    "mujoco/custom/tuple/element": "TupleElement",
    "mujoco/asset": "Asset",
    "mujoco/asset/mesh": "Mesh",
    "mujoco/asset/mesh/plugin": "PluginRef",
    "mujoco/asset/mesh/plugin/config": "Config",
    "mujoco/asset/hfield": "Hfield",
    "mujoco/asset/skin": "Skin",
    "mujoco/asset/skin/bone": "SkinBone",
    "mujoco/asset/texture": "Texture",
    "mujoco/asset/material": "Material",
    "mujoco/asset/material/layer": "MaterialLayer",
    "mujoco/asset/model": "ModelAsset",
    "mujoco/body": "Body",
    "mujoco/body/inertial": "Inertial",
    "mujoco/body/joint": "Joint",
    "mujoco/body/freejoint": "FreeJoint",
    "mujoco/body/geom": "Geom",
    "mujoco/body/geom/plugin": "PluginRef",
    "mujoco/body/geom/plugin/config": "Config",
    "mujoco/body/attach": "Attach",
    "mujoco/body/site": "Site",
    "mujoco/body/camera": "Camera",
    "mujoco/body/light": "Light",
    "mujoco/body/plugin": "PluginRef",
    "mujoco/body/plugin/config": "Config",
    "mujoco/body/composite": "Composite",
    "mujoco/body/composite/joint": "CompositeJoint",
    "mujoco/body/composite/skin": "CompositeSkin",
    "mujoco/body/composite/geom": "CompositeGeom",
    "mujoco/body/composite/site": "CompositeSite",
    "mujoco/body/composite/plugin": "PluginRef",
    "mujoco/body/composite/plugin/config": "Config",
    "mujoco/body/flexcomp": "Flexcomp",
    "mujoco/body/flexcomp/edge": "FlexcompEdge",
    "mujoco/body/flexcomp/elasticity": "FlexElasticity",
    "mujoco/body/flexcomp/contact": "FlexContact",
    "mujoco/body/flexcomp/pin": "FlexcompPin",
    "mujoco/body/flexcomp/plugin": "PluginRef",
    "mujoco/body/flexcomp/plugin/config": "Config",
    "mujoco/deformable": "Deformable",
    "mujoco/deformable/flex": "Flex",
    "mujoco/deformable/flex/contact": "FlexContact",
    "mujoco/deformable/flex/edge": "FlexEdge",
    "mujoco/deformable/flex/elasticity": "FlexElasticity",
    "mujoco/deformable/skin": "Skin",
    "mujoco/deformable/skin/bone": "SkinBone",
    "mujoco/contact": "Contact",
    "mujoco/contact/pair": "Pair",
    "mujoco/contact/exclude": "Exclude",
    "mujoco/equality": "Equality",
    "mujoco/equality/connect": "Connect",
    "mujoco/equality/weld": "Weld",
    "mujoco/equality/joint": "EqualityJoint",
    "mujoco/equality/tendon": "EqualityTendon",
    "mujoco/equality/flex": "EqualityFlex",
    "mujoco/equality/flexvert": "Flexvert",
    "mujoco/equality/flexstrain": "Flexstrain",
    "mujoco/tendon": "Tendon",
    "mujoco/tendon/spatial": "Spatial",
    "mujoco/tendon/spatial/site": "SpatialSite",
    "mujoco/tendon/spatial/geom": "SpatialGeom",
    "mujoco/tendon/spatial/pulley": "Pulley",
    "mujoco/tendon/fixed": "Fixed",
    "mujoco/tendon/fixed/joint": "FixedJoint",
    "mujoco/actuator": "Actuator",
    "mujoco/actuator/general": "ActuatorGeneral",
    "mujoco/actuator/motor": "Motor",
    "mujoco/actuator/position": "Position",
    "mujoco/actuator/velocity": "Velocity",
    "mujoco/actuator/intvelocity": "IntVelocity",
    "mujoco/actuator/damper": "Damper",
    "mujoco/actuator/cylinder": "Cylinder",
    "mujoco/actuator/muscle": "Muscle",
    "mujoco/actuator/adhesion": "Adhesion",
    "mujoco/actuator/dcmotor": "DcMotor",
    "mujoco/actuator/plugin": "ActuatorPlugin",
    "mujoco/actuator/plugin/config": "Config",
    "mujoco/sensor": "Sensor",
    "mujoco/sensor/contact": "SensorContact",
    "mujoco/sensor/plugin": "SensorPlugin",
    "mujoco/sensor/plugin/config": "Config",
    "mujoco/sensor/user": "SensorUser",
    "mujoco/keyframe": "Keyframe",
    "mujoco/keyframe/key": "Key",
}

# Child-list field names, keyed by (parent IDL element, child IDL element).
# Absence -> a sensible default derived from the child element name.
CHILD_NAMES = {
    ("Model", "Body"): "worldbody",
    ("Body", "Body"): "bodies",
    ("Body", "Geom"): "geoms",
    ("Body", "Joint"): "joints",
    ("Body", "Site"): "sites",
    ("Body", "Camera"): "cameras",
    ("Body", "Light"): "lights",
    ("Body", "Frame"): "frames",
    ("Body", "FreeJoint"): "freejoint",
    ("Body", "Inertial"): "inertial",
    ("Body", "PluginRef"): "plugin",
    ("Default", "Default"): "subclasses",
}

# Ordered heterogeneous child lists (union child lists). MJCF has sections whose
# element ids AND data addresses follow interleaved *document order across
# different tags*: actuator ids, sensor ids + sensor_adr, equality ids, tendon
# ids, and -- crucially -- the site/geom/pulley path of a spatial tendon, whose
# interleave IS the routing. A per-type child list cannot express that, so each
# of these container elements' per-tag child lists collapse into one union child
# list whose members preserve source order.
#   container element -> (union name, collapsed child-list field name)
UNION_CHILD_LISTS = OrderedDict([
    ("Actuator", ("ActuatorAny", "actuators")),
    ("Sensor", ("SensorAny", "sensors")),
    ("Equality", ("EqualityAny", "equalities")),
    ("Tendon", ("TendonAny", "tendons")),
    ("Spatial", ("PathItemAny", "path")),
])

# Elements that mix in Posed (pos + orientation variant).
POSED = {"Body", "Geom", "Site", "Camera", "Frame"}

# ref<T> targets, keyed by (IDL element, xml attr). `class` is handled globally.
REFS = {
    ("Geom", "material"): "Material",
    ("Geom", "mesh"): "Mesh",
    ("Geom", "hfield"): "Hfield",
    ("Site", "material"): "Site",  # placeholder, overwritten below
    ("Camera", "target"): "Body",
    ("Light", "target"): "Body",
    ("Light", "texture"): "Texture",
    ("Material", "texture"): "Texture",
    ("MaterialLayer", "texture"): "Texture",
    ("Mesh", "material"): "Material",
    ("Skin", "material"): "Material",
    ("Pair", "geom1"): "Geom",
    ("Pair", "geom2"): "Geom",
    ("Exclude", "body1"): "Body",
    ("Exclude", "body2"): "Body",
    ("Connect", "body1"): "Body",
    ("Connect", "body2"): "Body",
    ("Connect", "site1"): "Site",
    ("Connect", "site2"): "Site",
    ("Weld", "body1"): "Body",
    ("Weld", "body2"): "Body",
    ("Weld", "site1"): "Site",
    ("Weld", "site2"): "Site",
    ("EqualityJoint", "joint1"): "Joint",
    ("EqualityJoint", "joint2"): "Joint",
    ("EqualityTendon", "tendon1"): "TendonAny",
    ("EqualityTendon", "tendon2"): "TendonAny",
    ("EqualityFlex", "flex"): "Flex",
    ("Flexvert", "flex"): "Flex",
    ("Flexstrain", "flex"): "Flex",
    ("SpatialSite", "site"): "Site",
    ("SpatialGeom", "geom"): "Geom",
    ("SpatialGeom", "sidesite"): "Site",
    ("FixedJoint", "joint"): "Joint",
    ("Spatial", "material"): "Material",
    ("Fixed", "material"): "Material",
    ("TendonDefault", "material"): "Material",
    ("PluginRef", "instance"): "PluginInstance",
    ("ActuatorPlugin", "instance"): "PluginInstance",
    ("SensorPlugin", "instance"): "PluginInstance",
    ("Attach", "model"): "ModelAsset",
    # Rendering material references, same as Geom/Site/Mesh/Skin above -- these
    # were simply absent, leaving the schema inconsistent (a Flex's `material`
    # named a Material but typed as a bare string).
    ("CompositeSkin", "material"): "Material",
    ("CompositeGeom", "material"): "Material",
    ("CompositeSite", "material"): "Material",
    ("Flexcomp", "material"): "Material",
    ("Flex", "material"): "Material",
    # Sensors that measure an actuator, by name.
    ("Actuatorpos", "actuator"): "ActuatorAny",
    ("Actuatorvel", "actuator"): "ActuatorAny",
    ("Actuatorfrc", "actuator"): "ActuatorAny",
    # Sensors that measure a body subtree, by name. ActuatorGeneral/Adhesion
    # already typed their `body`; these three did not.
    ("Subtreecom", "body"): "Body",
    ("Subtreelinvel", "body"): "Body",
    ("Subtreeangmom", "body"): "Body",
    # <bone body="..."> names a body of this model.
    ("SkinBone", "body"): "Body",
    # NOT here, deliberately:
    #   ("Attach", "body") names a body inside the *referenced model asset*, not
    #     this model -- ref<Body> would resolve against the local tree and be
    #     wrong. It needs a ref-into-another-model concept.
    #   ("Flex", "body") is a space-separated LIST of body names, not one ref.
    #   sensor objtype/objname pairs carry the target type in a sibling
    #     attribute; Ref<T>'s phantom type cannot express a runtime type.
}
# material's `material` attr does not exist; the Site.material placeholder above
# is corrected here to keep the table declarative and free of ordering hazards.
REFS[("Site", "material")] = "Material"
_ACTUATOR_TRANSMISSION = {
    "joint": "Joint",
    "jointinparent": "Joint",
    "site": "Site",
    "refsite": "Site",
    "tendon": "TendonAny",
    "slidersite": "Site",
    "cranksite": "Site",
    "body": "Body",
}

# Variable-arity attribute ranges (Q-ARITY). Solver pairs, gear and fluid coefs
# have consistent bounds everywhere and are keyed by attr; `size`/`friction`
# mean different things per element and are keyed by (element, attr).
GLOBAL_ARITY = {
    "gear": (0, 6),
    "solref": (0, 2),
    "solimp": (0, 5),
    "solreflimit": (0, 2),
    "solimplimit": (0, 5),
    "solreffriction": (0, 2),
    "solimpfriction": (0, 5),
    "solreffix": (0, 2),
    "solimpfix": (0, 5),
    "fluidcoef": (0, 5),
}
ELEM_ARITY = {
    ("Geom", "size"): (0, 3),
    ("Geom", "friction"): (1, 3),
    ("Site", "size"): (0, 3),
    ("CompositeGeom", "size"): (0, 3),
    ("CompositeGeom", "friction"): (1, 3),
    ("CompositeSite", "size"): (0, 3),
    ("Flex", "friction"): (1, 3),
    ("FlexContact", "friction"): (1, 3),
    ("FlexcompContact", "friction"): (1, 3),
}

# unit=angle fields (Q-ANGLE). Keyed by (IDL element, attr). Orientation is
# canonicalized to quat at read (Q-ORIENT, Wave A), so authored euler/axisangle
# angles are consumed by the resolver, not stored -- no unit=angle field remains
# for them. Camera.fovy is NEVER subject to compiler.angle (it is degrees always,
# a length for orthographic cameras -- mjmodel.h:501), so it is not listed (R2).
UNIT_ANGLE = {
    ("Joint", "range"),
    ("Joint", "ref"),
    ("Joint", "springref"),
    ("CompositeJoint", "range"),
}

# Fallback attribute types for attributes not modelled by a per-family mjSpec
# struct (chiefly the ~50 typed sensors and the procedural blocks). Consulted
# only after the struct lookup fails, so struct-derived types always win. Types
# here are consistent wherever the attribute name appears.
GLOBAL_ATTR_TYPE = {
    "nsample": "int32", "delay": "double", "noise": "double", "cutoff": "double",
    "interval": "double[2]", "user": "double[]", "num": "int32", "dim": "int32",
    "prm": "double[]", "coef": "double", "polycoef": "double[]", "divisor": "double",
    "prefix": "string", "content_type": "string", "file": "string", "count": "int32",
    "spacing": "double[3]", "point": "double[]", "mass": "double", "scale": "double[3]",
    "rigid": "bool", "origin": "double[3]", "inertiabox": "double", "curve": "string",
    "initial": "string", "offset": "double[3]", "anchor": "double[3]",
    "relpose": "double[7]", "torquescale": "double", "subgrid": "int32",
    "key": "string", "value": "string", "plugin": "string", "objtype": "string",
    "objname": "string", "reftype": "string", "refname": "string", "actuator": "string",
    "data": "string", "vertex": "double[]", "normal": "double[]", "texcoord": "float[]",
    "face": "int32[]", "params": "double[]", "elevation": "double[]",
    "bindpos": "double[3]", "bindquat": "double[4]", "vertid": "int32[]",
    "vertweight": "float[]", "id": "int32[]", "grid": "int32[]", "gridrange": "int32[]",
    "cell": "int32", "stiffness": "double", "damping": "double", "young": "double",
    "poisson": "double", "thickness": "double", "solreffix": "double[0..2]",
    "solimpfix": "double[0..5]", "pos": "double[3]", "quat": "double[4]",
    "axisangle": "double[4]", "xyaxes": "double[6]", "zaxis": "double[3]",
    "euler": "double[3]", "axis": "double[3]", "range": "double[2]", "margin": "double",
    "frictionloss": "double", "armature": "double", "group": "int32", "kind": "string",
    "directional": "bool", "body": "string", "active": "bool",
    "solref": "double[0..2]", "solimp": "double[0..5]",
    "solreflimit": "double[0..2]", "solimplimit": "double[0..5]",
    "solreffriction": "double[0..2]", "solimpfriction": "double[0..5]",
}
# Typed cross-references for reference-slot attributes whose target element is
# unambiguous (DR-8). Fallback after explicit REFS; scopes the sensor object
# slots and similar. `body`, `actuator`, `objname` stay names (ambiguous target).
GLOBAL_REF = {
    "site": "Site", "joint": "Joint", "tendon": "TendonAny", "geom": "Geom",
    "camera": "Camera", "geom1": "Geom", "geom2": "Geom", "joint1": "Joint",
    "joint2": "Joint", "tendon1": "TendonAny", "tendon2": "TendonAny",
    "body1": "Body", "body2": "Body", "subtree1": "Body", "subtree2": "Body",
    "mesh": "Mesh",
}

# The single genuine rename: `class` is a C++ reserved word.
CLASS_FIELD = "dclass"

# Elements whose XML tag is a reused single-attr plugin attachment stub.
# All share {plugin, instance}.
PLUGIN_REF_ATTRS = ["plugin", "instance"]

# Elements reached from `default` that reuse a primary element definition.
DEFAULT_REUSE = {
    "mesh": "Mesh",
    "material": "Material",
    "joint": "Joint",
    "geom": "Geom",
    "site": "Site",
    "camera": "Camera",
    "light": "Light",
    "pair": "Pair",
    "equality": "EqualityDefault",
    "tendon": "TendonDefault",
    "general": "ActuatorGeneral",
    "motor": "Motor",
    "position": "Position",
    "velocity": "Velocity",
    "intvelocity": "IntVelocity",
    "damper": "Damper",
    "cylinder": "Cylinder",
    "muscle": "Muscle",
    "adhesion": "Adhesion",
    "dcmotor": "DcMotor",
}

# Typed actuator-shortcut parameters that lower to gain/bias/dyn prm at compile
# (Q-ACT): stored named, per spelling. attr -> (idl type, doc).
ACTUATOR_SHORTCUT = {
    "kp": ("double", "position feedback gain"),
    "kv": ("double", "velocity feedback gain"),
    "dampratio": ("double", "damping ratio, in units of critical damping"),
    "timeconst": ("double", "time constant of the activation dynamics"),
    "area": ("double", "cylinder area (alternative to diameter)"),
    "diameter": ("double", "cylinder diameter (alternative to area)"),
    "bias": ("double[3]", "cylinder bias parameters"),
    "gain": ("double", "adhesion gain"),
    "force": ("double", "peak active force (muscle); <0: automatic"),
    "scale": ("double", "muscle force = scale / acc0"),
    "lmin": ("double", "lower bound on muscle operating length"),
    "lmax": ("double", "upper bound on muscle operating length"),
    "vmax": ("double", "shortening velocity at which force drops to zero"),
    "fpmax": ("double", "peak passive force at length lmax"),
    "fvmax": ("double", "active force at saturating lengthening velocity"),
    "range": ("double[2]", "muscle operating length range"),
    "motorconst": ("double", "motor torque constant"),
    "resistance": ("double", "armature resistance"),
    "nominal": ("double", "nominal supply voltage"),
    "saturation": ("double", "voltage saturation"),
    "inductance": ("double", "armature inductance"),
    "cogging": ("double", "cogging torque amplitude"),
    "controller": ("double[3]", "controller gains"),
    "thermal": ("double[2]", "thermal model parameters"),
    "lugre": ("double[5]", "LuGre friction parameters"),
}

# Config-block attribute types where the mjSpec struct does not model the XML
# attribute (compiler/option/size/visual/statistic and a few stragglers).
# (IDL element, attr) -> idl type string. Anything not listed falls to a
# per-element default (see ELEMENT_DEFAULT_TYPE) or the double heuristic.
CONFIG_TYPES = {
    ("Model", "model"): "string",
    ("Compiler", "strippath"): "bool",
    ("Compiler", "assetdir"): "string",
    ("Compiler", "meshdir"): "string",
    ("Compiler", "texturedir"): "string",
    ("Compiler", "eulerseq"): "string",
    ("Compiler", "autolimits"): "bool",
    ("Compiler", "balanceinertia"): "bool",
    ("Compiler", "fitaabb"): "bool",
    ("Compiler", "discardvisual"): "bool",
    ("Compiler", "usethread"): "bool",
    ("Compiler", "fusestatic"): "bool",
    ("Compiler", "saveinertial"): "bool",
    ("Compiler", "alignfree"): "bool",
    ("Compiler", "inertiagrouprange"): "int32[2]",
    ("Compiler", "boundmass"): "double",
    ("Compiler", "boundinertia"): "double",
    ("Compiler", "settotalmass"): "double",
    ("Option", "gravity"): "double[3]",
    ("Option", "wind"): "double[3]",
    ("Option", "magnetic"): "double[3]",
    ("Option", "o_solref"): "double[2]",
    ("Option", "o_solimp"): "double[5]",
    ("Option", "o_friction"): "double[5]",
    ("Option", "iterations"): "int32",
    ("Option", "ls_iterations"): "int32",
    ("Option", "noslip_iterations"): "int32",
    ("Option", "ccd_iterations"): "int32",
    ("Option", "sdf_iterations"): "int32",
    ("Option", "sdf_initpoints"): "int32",
    ("Option", "actuatorgroupdisable"): "int32[]",
    ("Size", "memory"): "string",
    ("Size", "njmax"): "int32",
    ("Size", "nconmax"): "int32",
    ("Size", "nstack"): "int32",
    ("Size", "nuserdata"): "int32",
    ("Size", "nkey"): "int32",
    ("Statistic", "center"): "double[3]",
    ("VisualGlobal", "cameraid"): "int32",
    ("VisualGlobal", "orthographic"): "bool",
    ("VisualGlobal", "offwidth"): "int32",
    ("VisualGlobal", "offheight"): "int32",
    ("VisualGlobal", "ellipsoidinertia"): "bool",
    ("VisualGlobal", "bvactive"): "bool",
    ("VisualGlobal", "realtime"): "double",
    ("VisualQuality", "shadowsize"): "int32",
    ("VisualQuality", "offsamples"): "int32",
    ("VisualQuality", "numslices"): "int32",
    ("VisualQuality", "numstacks"): "int32",
    ("VisualQuality", "numquads"): "int32",
    ("VisualHeadlight", "ambient"): "float[3]",
    ("VisualHeadlight", "diffuse"): "float[3]",
    ("VisualHeadlight", "specular"): "float[3]",
    ("VisualHeadlight", "active"): "int32",
    ("VisualMap", "actuatortendon"): "double",
    ("VisualScale", "frustum"): "double",
    ("LengthRange", "useexisting"): "bool",
    ("LengthRange", "uselimit"): "bool",
    ("LengthRange", "accel"): "double",
    ("LengthRange", "maxforce"): "double",
    ("LengthRange", "timeconst"): "double",
    ("LengthRange", "timestep"): "double",
    ("LengthRange", "inttotal"): "double",
    ("LengthRange", "interval"): "double",
    ("LengthRange", "tolrange"): "double",
}
# Per-element fallbacks for whole config sub-blocks (colours, scales, maps).
ELEMENT_DEFAULT_TYPE = {
    "Flag": "bool",
    "VisualScale": "double",
    "VisualMap": "double",
    "VisualRgba": "float[4]",
    "Statistic": "double",
    "VisualGlobal": "double",
    "VisualQuality": "int32",
    "Size": "int32",
}


# --------------------------------------------------------------------------- #
# Snapshot loading + helpers                                                   #
# --------------------------------------------------------------------------- #
def load(name):
    with open(os.path.join(SNAP, name), "r", encoding="utf-8") as fh:
        return json.load(fh)


def pascal(tag):
    return "".join(p[:1].upper() + p[1:] for p in tag.replace("-", "_").split("_"))


_CTYPE_PRIM = {
    "int": "int32",
    "mjtByte": "bool",
    "bool": "bool",
    "double": "double",
    "mjtNum": "double",
    "float": "float",
    "mjtSize": "uint64",
    "uint64_t": "uint64",
    "mjString": "string",
    "mjStringVec": "string[]",
    "mjDoubleVec": "double[]",
    "mjFloatVec": "float[]",
    "mjIntVec": "int32[]",
    "mjByteVec": "int32[]",
    "mjIntVecVec": "int32[]",
    "mjFloatVecVec": "float[]",
}
_CTYPE_ENUM = {
    "mjtGeom": "GeomType",
    "mjtJoint": "JointType",
    "mjtTexture": "TextureType",
    "mjtLightType": "LightType",
    "mjtCamLight": "CamLightMode",
    "mjtProjection": "CameraProjection",
    "mjtGain": "GainType",
    "mjtBias": "BiasType",
    "mjtDyn": "DynType",
    "mjtMeshInertia": "MeshInertia",
    "mjtColorSpace": "ColorSpace",
    "mjtDataType": "DataType",
    "mjtStage": "NeedStage",
    "mjtSleepPolicy": "BodySleep",
}


def struct_type(field, arity_override):
    """IDL type string for an mjSpec struct field, honouring an arity override."""
    ctype = field["ctype"]
    arr = field["array"]
    if arity_override is not None:
        lo, hi = arity_override
        base = _CTYPE_PRIM.get(ctype, "double")
        base = base[:-2] if base.endswith("[]") else base
        return f"{base}[{lo}..{hi}]"
    if ctype in _CTYPE_ENUM:
        return _CTYPE_ENUM[ctype]
    if arr:
        n = arr["value"]
        if ctype == "char":
            return "string"  # char[N] buffers are strings (eulerseq, gridlayout)
        base = _CTYPE_PRIM.get(ctype, "double")
        base = base[:-2] if base.endswith("[]") else base
        return f"{base}[{n}]"
    return _CTYPE_PRIM.get(ctype, "double")


# Sentinel/opaque default markers that must NOT become IDL defaults (DR-1).
def real_default(spec, idl_type):
    """Render a snapshot default as an IDL default clause, or '' if none/sentinel."""
    if spec is None:
        return ""
    if isinstance(spec, dict):
        if "value" in spec:  # enum constant, handled by the caller
            return ""
        return ""  # sentinel / opaque / unset_by_memset
    if isinstance(spec, list):
        if any(isinstance(x, dict) for x in spec):
            return ""  # sentinel-bearing array
        if idl_type and idl_type.endswith("]") and ".." not in idl_type:
            # fixed array: only emit when length matches the declared extent
            try:
                n = int(idl_type[idl_type.index("[") + 1 : -1])
            except ValueError:
                return ""
            if len(spec) != n:
                return ""
        vals = ", ".join(_fmt_num(x) for x in spec)
        return f" = {{{vals}}}"
    if isinstance(spec, str):
        return ""  # string defaults are authoring conveniences; omit (DR-1)
    if idl_type == "bool":
        return f" = {'true' if spec else 'false'}"
    return f" = {_fmt_num(spec)}"


def _fmt_num(x):
    if isinstance(x, bool):
        return "true" if x else "false"
    if isinstance(x, float) and x.is_integer():
        return str(int(x))
    return str(x)


# --------------------------------------------------------------------------- #
# Field + element model                                                        #
# --------------------------------------------------------------------------- #
class Field:
    __slots__ = ("name", "type", "annots", "default", "doc")

    def __init__(self, name, type_, annots=None, default="", doc=None):
        self.name = name
        self.type = type_
        self.annots = annots or {}
        self.default = default
        self.doc = doc


class Element:
    def __init__(self, name, tag):
        self.name = name
        self.tag = tag
        self.uses = []
        self.fields = []
        self.children = []  # (field_name, target_element, cardinality)


CARD = {"*": "*", "?": "?", "!": "!", "R": "*"}


def build():
    mjcf = load("mjcf_schema.json")
    fields_snap = load("mjspec_fields.json")
    defaults_snap = load("spec_defaults.json")["defaults"]
    keyword_maps = mjcf["keyword_maps"]

    structs = {s["name"]: {f["name"]: f for f in s["fields"]} for s in fields_snap["structs"]}

    # element IDL name -> mjSpec struct name (for types/comments/defaults).
    elem_struct = {
        "Compiler": "mjsCompiler",
        "Body": "mjsBody",
        "Inertial": "mjsBody",
        "Joint": "mjsJoint",
        "FreeJoint": "mjsJoint",
        "Geom": "mjsGeom",
        "CompositeGeom": "mjsGeom",
        "Site": "mjsSite",
        "CompositeSite": "mjsSite",
        "SpatialSite": "mjsSite",
        "Camera": "mjsCamera",
        "Light": "mjsLight",
        "Frame": "mjsFrame",
        "Mesh": "mjsMesh",
        "Hfield": "mjsHField",
        "Skin": "mjsSkin",
        "CompositeSkin": "mjsSkin",
        "Texture": "mjsTexture",
        "Material": "mjsMaterial",
        "Pair": "mjsPair",
        "Exclude": "mjsExclude",
        "Connect": "mjsEquality",
        "Weld": "mjsEquality",
        "EqualityJoint": "mjsEquality",
        "EqualityTendon": "mjsEquality",
        "EqualityFlex": "mjsEquality",
        "Flexvert": "mjsEquality",
        "Flexstrain": "mjsEquality",
        "Spatial": "mjsTendon",
        "Fixed": "mjsTendon",
        "TendonDefault": "mjsTendon",
        "EqualityDefault": "mjsEquality",
        "FlexContact": "mjsFlex",
        "FlexcompContact": "mjsFlex",
        "FlexElasticity": "mjsFlex",
        "FlexcompElasticity": "mjsFlex",
        "ActuatorGeneral": "mjsActuator",
        "Motor": "mjsActuator",
        "Position": "mjsActuator",
        "Velocity": "mjsActuator",
        "IntVelocity": "mjsActuator",
        "Damper": "mjsActuator",
        "Cylinder": "mjsActuator",
        "Muscle": "mjsActuator",
        "Adhesion": "mjsActuator",
        "DcMotor": "mjsActuator",
        "ActuatorPlugin": "mjsActuator",
        "Numeric": "mjsNumeric",
        "Text": "mjsText",
        "Tuple": "mjsTuple",
        "Key": "mjsKey",
        "Flex": "mjsFlex",
        "Composite": "mjsFlex",
        "Flexcomp": "mjsFlex",
    }
    # attr -> struct field-name alias, used only for type/comment lookup (the
    # emitted field keeps the XML attr name, so coverage stays trivial).
    alias = {
        "mesh": "meshname",
        "hfield": "hfieldname",
        "user": "userdata",
        "diaginertia": "inertia",
        "vertex": "vert",
        "fluidcoef": "fluid_coefs",
        "target": "targetbody",
        "focal": "focal_length",
        "focalpixel": "focal_pixel",
        "principal": "principal_length",
        "principalpixel": "principal_pixel",
        "sensorsize": "sensor_size",
        "actuatorfrclimited": "actfrclimited",
        "actuatorfrcrange": "actfrcrange",
        "actuatorgravcomp": "actgravcomp",
        "solreflimit": "solref_limit",
        "solimplimit": "solimp_limit",
        "solreffriction": "solref_friction",
        "solimpfriction": "solimp_friction",
        "geom1": "geomname1",
        "geom2": "geomname2",
        "body1": "bodyname1",
        "body2": "bodyname2",
    }

    used_enums = set()

    def enum_ref(name):
        used_enums.add(name)
        return name

    def make_field(elem_name, attr):
        """Resolve one XML attribute into a Field, or None to skip (variant-owned)."""
        # Element identity + class references (stored in mjsElement, not the
        # per-family struct, so they need explicit handling).
        if attr == "name":
            return Field("name", "string", doc="element name")
        if attr == "childclass":
            return Field("childclass", "ref<Default>",
                         doc="default class applied to descendants")
        if attr == "shellinertia":
            return Field("shellinertia", "bool",
                         doc="use shell (surface) instead of volume inertia")
        if elem_name == "Texture" and attr.startswith("file") and attr != "file":
            return Field(attr, "string", doc="image file for one cube face")
        # actuator transmission targets are refs on every actuator spelling
        if elem_struct.get(elem_name) == "mjsActuator" and attr in _ACTUATOR_TRANSMISSION:
            return Field(attr, f"ref<{_ACTUATOR_TRANSMISSION[attr]}>", doc=_doc(elem_name, attr))
        # `class` -> the one genuine rename.
        if attr == "class":
            if elem_name == "Default":
                return Field(CLASS_FIELD, "string", {"xml": "class"},
                             doc="name of this default class")
            return Field(CLASS_FIELD, "ref<Default>", {"xml": "class"},
                         doc="default class")
        # explicit ref targets
        if (elem_name, attr) in REFS:
            return Field(attr, f"ref<{REFS[(elem_name, attr)]}>", doc=_doc(elem_name, attr))
        # enum assignment
        enum = ATTR_ENUM.get((elem_name, attr))
        if enum is None and attr in _TRISTATE_ATTRS:
            enum = "TriState"
        if enum is None:
            enum = _GLOBAL_ENUM.get(attr)
        if enum is not None:
            if (elem_name, attr) in ENUM_LIST_ATTRS:
                # MapValues bitmask: a space-separated keyword set, no scalar
                # default (the bitmask default is a read-time concern).
                return Field(attr, enum_ref(enum) + "[]", doc=_doc(elem_name, attr))
            return Field(attr, enum_ref(enum), doc=_doc(elem_name, attr),
                         default=_enum_default(elem_name, attr, enum))
        # actuator shortcut params (Q-ACT)
        if attr in ACTUATOR_SHORTCUT and elem_struct.get(elem_name) == "mjsActuator":
            typ, doc = ACTUATOR_SHORTCUT[attr]
            return Field(attr, typ, doc=doc)
        # config-block curated types
        if (elem_name, attr) in CONFIG_TYPES:
            return Field(attr, CONFIG_TYPES[(elem_name, attr)], doc=_doc(elem_name, attr))
        # struct-derived type
        struct_name = elem_struct.get(elem_name)
        if struct_name:
            fname = alias.get(attr, attr)
            sf = structs[struct_name].get(fname)
            if sf is not None:
                arity = ELEM_ARITY.get((elem_name, attr)) or GLOBAL_ARITY.get(attr)
                typ = struct_type(sf, arity)
                dflt = _struct_default(struct_name, fname, typ)
                annots = {}
                if (elem_name, attr) in UNIT_ANGLE:
                    annots["unit"] = "angle"
                return Field(attr, typ, annots, dflt, sf["comment"] or None)
        # per-element default type (whole-block config elements: flags are all
        # bool, scale/rgba colours are all the same type -- this must win over
        # the global fallbacks so e.g. VisualRgba.camera stays a colour, not a ref)
        if elem_name in ELEMENT_DEFAULT_TYPE:
            return Field(attr, ELEMENT_DEFAULT_TYPE[elem_name], doc=_doc(elem_name, attr))
        # typed cross-reference fallback (unambiguous target)
        if attr in GLOBAL_REF:
            return Field(attr, f"ref<{GLOBAL_REF[attr]}>", doc=_doc(elem_name, attr))
        # consistent-type fallback (sensors, procedural blocks)
        if attr in GLOBAL_ATTR_TYPE:
            return Field(attr, GLOBAL_ATTR_TYPE[attr], doc=_doc(elem_name, attr))
        # last resort
        return Field(attr, "double", doc=_doc(elem_name, attr))

    def _doc(elem_name, attr):
        struct_name = elem_struct.get(elem_name)
        if struct_name:
            sf = structs[struct_name].get(alias.get(attr, attr))
            if sf and sf["comment"]:
                return sf["comment"]
        return None

    def _struct_default(struct_name, fname, idl_type):
        d = defaults_snap.get(struct_name, {})
        return real_default(d.get(fname), idl_type)

    def _enum_default(elem_name, attr, enum):
        struct_name = elem_struct.get(elem_name)
        if not struct_name:
            return ""
        sf_name = alias.get(attr, attr)
        spec = defaults_snap.get(struct_name, {}).get(sf_name)
        if isinstance(spec, dict) and "value" in spec:
            const = spec["value"]
            rev = {m["value"]: m["key"] for m in keyword_maps[ENUM_MAPS[enum]]}
            key = rev.get(const)
            if key is not None:
                member = _enum_member(key)
                return f" = {member}"
        return ""

    # Orientation input aliases (Q-ORIENT, Wave A): the four non-canonical MJCF
    # spellings a posed element still accepts on input. They are resolved into the
    # canonical `quat` field at parse end (folded compiler.angle/eulerseq), never
    # stored as their own field. Listed on the canonical field's `aliases`
    # annotation; `quat` itself is the field name / xml attribute.
    ORIENT_ALIASES = "euler axisangle xyaxes zaxis"
    # Attributes any posed element (Posed mixin) covers: pos + the canonical quat
    # + its four input-alias spellings.
    ORIENT_ARMS = ["quat", "axisangle", "xyaxes", "zaxis", "euler"]

    def variant_fields(elem_name, attrs):
        """Return (injected_fields, consumed_attr_set) for an element.

        Injected fields are canonical fields whose authored MJCF spellings a
        resolver folds at parse end (orientation -> quat, fullinertia ->
        diaginertia+iquat, Q-ORIENT/Q-INERTIA), plus the surviving DR-3 variants
        (GeomShape fromto, TextureSource). The consumed set is every MJCF
        attribute these fields cover, so `walk` emits no plain field for them.
        """
        vf, consumed = [], set()
        aset = set(attrs)
        if elem_name == "Inertial":
            # Inertial frame orientation is authored via quat/euler/... (same
            # spellings as a body); its canonical field is `iquat` (wire attr
            # `quat`). fullinertia resolves via eigendecomposition into
            # diaginertia + iquat, so it is an alias of the canonical diaginertia.
            vf.append(Field("iquat", "double[4]",
                            {"xml": "quat", "aliases": ORIENT_ALIASES,
                             "resolver": "orientation"},
                            doc="inertial frame orientation (canonical quat)"))
            consumed |= set(ORIENT_ARMS)
            vf.append(Field("diaginertia", "double[3]",
                            {"aliases": "fullinertia", "resolver": "inertia"},
                            doc="diagonal inertia (canonical; also accepts "
                                "fullinertia)"))
            consumed |= {"diaginertia", "fullinertia"}
        if elem_name in ("Geom", "Site") and "fromto" in aset:
            vf.append(Field("shape", "variant GeomShape",
                            doc="explicit size or fromto endpoints"))
            consumed.add("fromto")
        if elem_name == "Camera":
            # R1: the fovy/focal exclusion is not a real variant (fovy vs
            # sensorsize is, and lives class-splittable at compile). Both are plain
            # KEEP-semantics fields; the reader enforces the fovy+sensorsize error.
            vf.append(Field("fovy", "double",
                            doc="vertical field of view (deg; length if ortho)"))
            vf.append(Field("focal", "double[2]", doc="focal length (length)"))
            consumed |= {"fovy", "focal"}
        if elem_name == "Flexcomp":
            # Flexcomp keeps pos + origin (plain) and gains the canonical quat;
            # its four alt orientation spellings resolve into quat (Q-ORIENT #2).
            vf.append(Field("quat", "double[4]",
                            {"aliases": ORIENT_ALIASES, "resolver": "orientation"},
                            doc="orientation (canonical quat; also accepts "
                                "euler/axisangle/xyaxes/zaxis)"))
            consumed |= set(ORIENT_ARMS)
        if elem_name == "Texture":
            vf.append(Field("source", "variant TextureSource",
                            doc="builtin generator or image file"))
            consumed |= {"builtin", "file"}
        used_enums.add("TextureBuiltin")  # arm type
        return vf, consumed

    # ------------------------------------------------------------------- #
    # Walk the MJCF tree, minting elements.                               #
    # ------------------------------------------------------------------- #
    elements = OrderedDict()  # name -> Element

    def name_for(path, tag):
        return PATH_NAMES.get(path, pascal(tag))

    def walk(node, parent_path):
        tag = node["name"]
        path = tag if not parent_path else parent_path + "/" + tag
        ename = name_for(path, tag)

        if ename in elements:
            elem = elements[ename]  # reused definition; still recurse for children
            _wire_children(elem, node, path)
            return ename
        elem = Element(ename, tag)
        elements[ename] = elem

        if ename == "Default":
            _build_default(elem, node, path)
            return ename
        if ename == "PluginRef":
            elem.fields.append(Field("plugin", "string", doc="plugin name"))
            elem.fields.append(Field("instance", "ref<PluginInstance>",
                                     doc="plugin instance name"))
            return ename

        attrs = node["attributes"]
        vf, consumed = variant_fields(ename, attrs)
        if ename in POSED:
            elem.uses.append("Posed")
            consumed |= set(ORIENT_ARMS)
            consumed.add("pos")
        for attr in attrs:
            if attr in consumed:
                continue
            f = make_field(ename, attr)
            if f is not None:
                elem.fields.append(f)
        elem.fields.extend(vf)
        _wire_children(elem, node, path)
        return ename

    def _wire_children(elem, node, path):
        existing = {c[0] for c in elem.children}
        for child in node["children"]:
            cname = walk(child, path)
            fld = CHILD_NAMES.get((elem.name, cname))
            if fld is None:
                fld = _default_child_name(cname)
            if fld in existing:
                continue
            existing.add(fld)
            elem.children.append((fld, cname, CARD.get(child["typecode"], "*")))

    def _build_default(elem, node, path):
        elem.fields.append(Field(CLASS_FIELD, "string", {"xml": "class"},
                                 doc="name of this default class"))
        for child in node["children"]:
            tag = child["name"]
            target = DEFAULT_REUSE.get(tag)
            if target is None:
                continue
            # equality/tendon defaults have no primary element; mint them here.
            # everything else reuses a primary defined during the body/asset walk.
            if target in ("EqualityDefault", "TendonDefault"):
                walk(child, path)
            elem.children.append((tag, target, "?"))
        elem.children.append(("subclasses", "Default", "*"))

    root = mjcf["elements"]
    walk(root, "")

    # Body-in-body recursion. MuJoCo's MJCF[] table encodes the body tree via
    # the R type-code self-reference on the body row rather than an explicit
    # <body> child row (the reader recurses into nested <body> at
    # xml_native_reader.cc:3877-3925), so _wire_children -- which walks snapshot
    # child rows -- never adds it. Wire the recursion here as curation data.
    body = elements["Body"]
    body.children.append(("bodies", "Body", "*"))

    # Frame is first-class (persists in mjSpec) but validates against the body
    # row, so it is not a distinct MJCF tree node. Add it explicitly.
    frame = Element("Frame", "frame")
    frame.uses.append("Posed")
    frame.fields.append(Field("name", "string"))
    frame.fields.append(Field(CLASS_FIELD, "ref<Default>", {"xml": "class"},
                              doc="childclass applied to descendants"))
    elements["Frame"] = frame
    body.children.append(("frames", "Frame", "*"))

    # Replicate is a first-class pass-through element (plan DR-7 / Q-MACRO): the
    # bridge emits the tag and MuJoCo clones the enclosed subtree at compile. Like
    # frame it validates against the body row (mjXSchema::NameMatch,
    # xml_util.cc:492) rather than owning an MJCF[] row, so it is added
    # explicitly. Its control attributes come from the reader's replicate branch
    # (xml_native_reader.cc:3806-3874): count (3812, required), offset (3813),
    # euler (3814, accumulated as a rotation so unit=angle), sep (3815), and
    # childclass (3826). prefix is written in the corpus (and namespaces the
    # clones) but is not consumed by this vendored reader; it is modelled for
    # pass-through and round-trip fidelity.
    replicate = Element("Replicate", "replicate")
    replicate.fields.append(Field("count", "int32", {"required": True},
                                  doc="number of copies to generate"))
    replicate.fields.append(Field("offset", "double[3]",
                                  doc="position offset accumulated per copy"))
    replicate.fields.append(Field("euler", "double[3]", {"unit": "angle"},
                                  doc="rotation offset accumulated per copy"))
    replicate.fields.append(Field("sep", "string",
                                  doc="separator for the cloned element name suffix"))
    replicate.fields.append(Field("prefix", "string",
                                  doc="prefix applied to cloned element names"))
    replicate.fields.append(Field("childclass", "ref<Default>",
                                  doc="default class applied to descendants"))
    elements["Replicate"] = replicate
    body.children.append(("replicates", "Replicate", "*"))

    # Frame and Replicate both wrap body-context content, which the reader
    # resolves through recursive Body() calls (frame at xml_native_reader.cc:3803,
    # replicate at :3843). They therefore mirror Body's complete body-context
    # child set -- now including nested bodies, frames and replicates, and attach.
    for fld, cname, card in body.children:
        frame.children.append((fld, cname, card))
        replicate.children.append((fld, cname, card))

    # Collapse the interleaved-order container sections into union child lists.
    # The members are the container's per-tag child targets in source order.
    unions = OrderedDict()
    for container, (uname, fld) in UNION_CHILD_LISTS.items():
        elem = elements[container]
        members = []
        for _fname, target, _card in elem.children:
            if target not in members:
                members.append(target)
        unions[uname] = members
        elem.children = [(fld, uname, "*")]

    return elements, keyword_maps, used_enums, unions


_CHILD_SINGULAR = {"PluginRef": "plugin", "Config": "config", "Attach": "attach",
                   "SkinBone": "bones", "MaterialLayer": "layers"}


def _default_child_name(cname):
    if cname in _CHILD_SINGULAR:
        return _CHILD_SINGULAR[cname]
    base = cname[:1].lower() + cname[1:]
    return base + "s" if not base.endswith("s") else base


def _enum_member(key):
    """A legal IDL identifier for an enum member derived from an XML keyword."""
    m = key
    subs = {
        "2d": "twod",
        "false": "false_",
        "true": "true_",
        "0": "zero",
        "cos(s)": "coss",
        "sin(s)": "sins",
        "s": "s",
    }
    if m in subs:
        return subs[m]
    m = m.replace("-", "_").replace("+", "_")
    m = "".join(ch if (ch.isalnum() or ch == "_") else "_" for ch in m)
    if m and m[0].isdigit():
        m = "n" + m
    return m


# --------------------------------------------------------------------------- #
# Emit                                                                         #
# --------------------------------------------------------------------------- #
def render(elements, keyword_maps, used_enums, unions):
    out = []
    w = out.append
    w('mujoco_version "3.10.0"')
    w("")
    w("# " + "=" * 74)
    w("# ProtoSpec IDL schema for MuJoCo MJCF (generated by")
    w("# tools/bootstrap/draft_schema.py, then hand-curated -- see plan Section 5).")
    w("# " + "=" * 74)
    w("")

    # Enums
    w("# --- Enums (keyword tables from the MJCF mjMap[] surface) " + "-" * 18)
    w("")
    for name in sorted(used_enums):
        members = keyword_maps[ENUM_MAPS[name]]
        seen = {}
        w(f"enum {name} {{")
        for m in members:
            member = _enum_member(m["key"])
            if member in seen:  # (e.g. distinct XML keywords colliding) keep unique
                member = member + "_" + str(len(seen))
            seen[member] = True
            w(f'  {member:14} = "{m["key"]}"')
        w("}")
        w("")

    # Shape / texture variants (surviving DR-3 groups) + canonicalized orientation
    w("# --- Shape / texture variants (DR-3) + canonical orientation " + "-" * 14)
    w("")
    w("# Orientation is canonicalized to a single unit quaternion at read time")
    w("# (Q-ORIENT, docs/plan_canonicalization.md Wave A): the reader accepts all")
    w("# five MJCF spellings and folds euler/axisangle/xyaxes/zaxis into `quat`")
    w("# against the effective compiler.angle/eulerseq. The four non-canonical")
    w("# spellings are read-only input aliases (see the `aliases` annotation on")
    w("# each canonical `quat`/`iquat`); the writer emits `quat` only.")
    w("")
    w("struct Explicit { size : double[0..3] }")
    w("struct FromTo   { fromto : double[6] }")
    w("variant GeomShape {")
    w("  explicit : Explicit     # type-specific size, at the element's own pose")
    w("  fromto   : FromTo       # capsule/cylinder/box/ellipsoid endpoints")
    w("}")
    w("")
    w("struct TexFile { file : string }")
    w("variant TextureSource {")
    w("  builtin : TextureBuiltin   # procedural generator")
    w("  file    : TexFile          # single image file (buffer form is API-only)")
    w("}")
    w("")
    w("mixin Posed {")
    w("  pos  : double[3] = {0, 0, 0}   # position offset")
    w('  quat : double[4] (aliases="euler axisangle xyaxes zaxis", '
      "resolver=orientation)   # orientation (canonical quat; also accepts "
      "euler/axisangle/xyaxes/zaxis)")
    w("}")
    w("")

    # Unions (interleaved-order sections + any-tendon refs; see UNION_CHILD_LISTS)
    w("# --- Unions (ordered heterogeneous child lists / any-of refs) " + "-" * 13)
    w("")
    for uname, members in unions.items():
        w(f"union {uname} =")
        for i, m in enumerate(members):
            w(("    " if i == 0 else "  | ") + m)
        w("")

    # Elements, in tree order (Model first), section comment.
    w("# --- Elements " + "-" * 61)
    w("")
    for elem in elements.values():
        _render_element(w, elem)
    return "\n".join(out) + "\n"


def _render_element(w, elem):
    header = f"element {elem.name}"
    if elem.tag != elem.name.lower():
        header += f' (xml="{elem.tag}")'
    w(header + " {")
    for use in elem.uses:
        w(f"  use {use}")
    # column widths for tidy alignment
    fname_w = max([len(f.name) for f in elem.fields], default=0)
    for f in elem.fields:
        line = f"  {f.name:<{fname_w}} : {f.type}"
        if f.annots:
            parts = []
            for k in sorted(f.annots):
                v = f.annots[k]
                if k in ("xml", "aliases"):
                    parts.append(f'{k}="{v}"')  # string-valued annotations
                elif v is True:
                    parts.append(k)  # bare flag (required, element_text)
                else:
                    parts.append(f"{k}={v}")  # identifier-valued (resolver, unit)
            line += f" ({', '.join(parts)})"
        line += f.default
        if f.doc:
            line += f"   # {f.doc}"
        w(line)
    for fld, target, card in elem.children:
        w(f"  children {fld} : {target} {card}")
    w("}")
    w("")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true",
                    help="verify schema/mujoco.spec is up to date, do not write")
    args = ap.parse_args()

    elements, keyword_maps, used_enums, unions = build()
    text = render(elements, keyword_maps, used_enums, unions)

    if args.check:
        with open(OUT, "r", encoding="utf-8") as fh:
            current = fh.read()
        if current != text:
            sys.stderr.write("schema/mujoco.spec is out of date; re-run draft_schema.py\n")
            sys.exit(1)
        print("schema/mujoco.spec is up to date")
        return

    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    with open(OUT, "w", encoding="utf-8", newline="\n") as fh:
        fh.write(text)
    n_fields = sum(len(e.fields) for e in elements.values())
    n_children = sum(len(e.children) for e in elements.values())
    print(f"wrote {OUT}")
    print(f"  {len(elements)} elements, {n_fields} fields, "
          f"{n_children} child links, {len(used_enums)} enums, "
          f"{len(unions)} unions")


if __name__ == "__main__":
    main()
