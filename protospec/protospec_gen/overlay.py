"""ProtoSpec overlay: the facts `src/xml/mjcf.schema` deliberately does not state.

The schema is the single source of truth for the MJCF grammar -- elements,
attributes, types, arities, defaults, enums, namespaces, presence constraints,
and the mjSpec struct/field bindings. It says nothing about how URLab spells
those things in C++ or how a spec-ordered heterogeneous child list is assembled
from per-tag child declarations. The overlay records naming, list shaping and
the few type or semantic corrections that upstream metadata cannot express.
The direct UE XML gateway handles ordinary fields; MuJoCo's native compiler
owns its custom read/write facets. No entry changes what the grammar means.

The tables are drift-gated. :mod:`protospec_gen.frontend` fails loudly on an
entry naming an element, attribute, or enum the schema no longer declares, and
on a stale semantic override. A schema bump therefore surfaces unexpected
changes as named errors rather than silently stale output.
"""

from __future__ import annotations

# --------------------------------------------------------------------------- #
# Naming                                                                       #
# --------------------------------------------------------------------------- #
# The default C++ name of a schema element is its name in PascalCase with
# underscores removed (`sensor_contact` -> `SensorContact`). The entries below
# are the divergences: the root element, the visual sub-blocks and plugin
# spellings whose bare tag would be uninformative as a type name, and the
# context-disambiguated tags whose schema name reads better reordered. The
# schema's own `xml=` facet already disambiguates the tags; this only chooses
# the identifier.
ELEMENT_NAMES = {
    "mujoco": "Model",
    "lengthrange": "LengthRange",
    "freejoint": "FreeJoint",
    "intvelocity": "IntVelocity",
    "dcmotor": "DcMotor",
    "global": "VisualGlobal",
    "quality": "VisualQuality",
    "headlight": "VisualHeadlight",
    "map": "VisualMap",
    "scale": "VisualScale",
    "rgba": "VisualRgba",
    "layer": "MaterialLayer",
    "extension_plugin": "PluginDef",
    "instance": "PluginInstance",
    "plugin": "PluginRef",
    "bone": "SkinBone",
    "model": "ModelAsset",
    "elasticity": "FlexElasticity",
    "flexcomp_contact": "FlexContact",
    "pin": "FlexcompPin",
    "general": "ActuatorGeneral",
    "orientation": "OrientationActuator",
    "user": "SensorUser",
    "element": "TupleElement",
    "default_equality": "EqualityDefault",
    "default_tendon": "TendonDefault",
}

# Elements the object model does not materialize as a type. `worldbody` is the
# schema's alias row for the world-level body slot: it carries no attributes and
# no spec struct, and `mujoco`'s `child body R` is that slot. ProtoSpec models it
# as a Body child list under the `worldbody` tag (see CHILD_TAGS), so a separate
# type would be an empty duplicate of Body.
ELEMENT_SKIP = {
    "worldbody": "alias row for the world-level body slot; modelled as "
                 "Model.worldbody, a Body list under the worldbody tag",
}

# The default C++ name of a schema enum is its name in PascalCase. These are the
# divergences, all cases where the schema name is the MJCF attribute spelling
# rather than a type name.
ENUM_NAMES = {
    "angle": "AngleUnit",
    "fluidshape": "FluidShape",
    "bodysleep": "BodySleep",
    "jointtype": "JointType",
    "geomtype": "GeomType",
    "lighttype": "LightType",
    "colorspace": "ColorSpace",
    "datatype": "DataType",
    "meshinertia": "MeshInertia",
    "FalseTrueAuto": "TriState",
    "FalseAuto": "SimpleMode",
    "projection": "CameraProjection",
    "camlight": "CamLightMode",
    "texrole": "TexRole",
    "jacobian": "JacobianType",
    "solver": "SolverType",
    "equality": "EqualityType",
    "texture": "TextureType",
    "builtin": "TextureBuiltin",
    "mark": "TextureMark",
    "dyn": "DynType",
    "gain": "GainType",
    "bias": "BiasType",
    "interp": "InterpType",
    "stage": "NeedStage",
    "frameobj": "FrameObject",
    "condata": "ContactData",
    "raydata": "RayData",
    "camout": "CameraOutput",
    "reduce": "ContactReduce",
    "lrmode": "LRMode",
    "comp": "CompositeType",
    "jkind": "JointKind",
    "shape": "CurveShape",
    "fcomp": "FlexcompType",
    "fdof": "FlexDof",
    "flexself": "FlexSelfCollide",
    "elastic2d": "Elastic2D",
    "flexeq": "FlexEquality",
    "inputchart": "InputChart",
    "inputbit": "InputBit",
    "dcmotorinput": "DcMotorInput",
    "meshbuiltin": "MeshBuiltin",
}

# Enumerators for XML keywords that are not C++ identifiers. Keywords that are
# merely C++ keywords (`true`, `false`) are sanitized by the emitter's trailing
# underscore and need no entry.
ENUM_MEMBERS = {
    ("texture", "2d"): "twod",
    ("fdof", "2d"): "twod",
    ("shape", "s"): "s",
    ("shape", "cos(s)"): "cos_s",
    ("shape", "sin(s)"): "sin_s",
    ("shape", "0"): "zero",
}

# C++ field names for attributes whose schema name cannot be a member. Keyed by
# attribute name (all elements) or by (element, attribute). `class` is a C++
# keyword; `dclass` reads better than the emitter's `class_`.
ATTR_NAMES = {
    "class": "dclass",
}


# --------------------------------------------------------------------------- #
# Schema type corrections                                                      #
# --------------------------------------------------------------------------- #
# The attributes `src/xml/mjcf.schema` types differently from the way the engine
# actually reads them. MuJoCo owns the schema and ProtoSpec never edits it, so
# the correction lives here -- but as a PAIR, not a replacement: `observed` is
# the declaration the schema carries today and `corrected` is what ProtoSpec
# reads instead, each as (type, target, arity, required) with arity the
# (low, high) token count -- (1, 1) for a scalar.
#
# The frontend refuses to apply an override whose `observed` no longer matches
# the schema in every one of those four positions, and refuses to keep one that
# never applied. So the day upstream retypes one of these -- whether it fixes
# it, changes it, or drops the attribute -- the generator fails naming the
# entry, instead of silently masking a now-correct declaration. An override
# cannot rot into a lie.
#
# key: (element, attribute) -> {observed, corrected, reason}
ATTR_TYPE_OVERRIDES = {
    ("mujoco", "model"): {
        "observed": ("ref", "model", (1, 1), False),
        "corrected": ("string", None, (1, 1), False),
        "reason": "<mujoco model=> is the model's own name, not a reference to "
                  "a nested-model asset: mjXReader::Parse puts it in "
                  "spec->modelname (xml_native_reader.cc:210-213), "
                  "XMLreference types it `string, \"MuJoCo Model\"`, and of the "
                  "64 corpus models that set it, none names a <model> asset.",
    },
    ("muscle", "timeconst"): {
        "observed": ("double", None, (1, 1), False),
        "corrected": ("double", None, (2, 2), False),
        "reason": "activation and de-activation are two separate constants: "
                  "the reader takes two tokens (xml_native_reader.cc:1383), "
                  "mjs_setToMuscle's parameter is double[2], and XMLreference "
                  "types it `real(2), \"0.01 0.04\"`. No corpus model authors "
                  "it, which is why nothing else catches the arity.",
    },
}


# --------------------------------------------------------------------------- #
# Child lists                                                                  #
# --------------------------------------------------------------------------- #
# MJCF sections whose spec order across different repeatable tags is semantic.
# Only the public list and union names are UE policy. The frontend derives union
# membership, in schema order, from all `*`/`R` child declarations and expands
# upstream `alias=` targets (such as frame/replicate -> body). A newly declared
# sensor or actuator therefore joins its ordered list without an overlay edit.
INTERLEAVE = {
    "body": [("subtree", "BodyChildAny")],
    "tendon": [("tendons", "TendonAny")],
    "spatial": [("path", "PathItemAny")],
    "equality": [("equalities", "EqualityAny")],
    "actuator": [("actuators", "ActuatorAny")],
    "sensor": [("sensors", "SensorAny")],
}

# Names for the homogeneous child lists. The default is the child element's C++
# name with a lowercased first letter; these are the divergences, which are the
# established plural spellings.
CHILD_NAMES = {
    ("mujoco", "compiler"): "compilers",
    ("mujoco", "option"): "options",
    ("mujoco", "size"): "sizes",
    ("mujoco", "statistic"): "statistics",
    ("mujoco", "visual"): "visuals",
    ("mujoco", "default"): "defaults",
    ("mujoco", "extension"): "extensions",
    ("mujoco", "asset"): "assets",
    ("mujoco", "body"): "worldbody",
    ("mujoco", "deformable"): "deformables",
    ("mujoco", "contact"): "contacts",
    ("mujoco", "tendon"): "tendons",
    ("mujoco", "equality"): "equalities",
    ("mujoco", "actuator"): "actuators",
    ("mujoco", "sensor"): "sensors",
    ("mujoco", "custom"): "customs",
    ("mujoco", "keyframe"): "keyframes",
    ("compiler", "lengthrange"): "lengthRanges",
    ("option", "flag"): "flags",
    ("visual", "global"): "visualGlobals",
    ("visual", "quality"): "visualQualities",
    ("visual", "headlight"): "visualHeadlights",
    ("visual", "map"): "visualMaps",
    ("visual", "scale"): "visualScales",
    ("visual", "rgba"): "visualRgbas",
    ("default", "default"): "subclasses",
    ("default", "default_equality"): "equality",
    ("default", "default_tendon"): "tendon",
    ("default", "general"): "general",
    ("default", "orientation"): "orientation",
    ("default", "intvelocity"): "intvelocity",
    ("default", "dcmotor"): "dcmotor",
    ("extension", "extension_plugin"): "pluginDefs",
    ("extension_plugin", "instance"): "pluginInstances",
    ("asset", "mesh"): "meshes",
    ("asset", "hfield"): "hfields",
    ("asset", "skin"): "skins",
    ("asset", "texture"): "textures",
    ("asset", "material"): "materials",
    ("asset", "model"): "modelAssets",
    ("skin", "bone"): "bones",
    ("mesh", "plugin"): "plugin",
    ("geom", "plugin"): "plugin",
    ("composite", "plugin"): "plugin",
    ("flexcomp", "plugin"): "plugin",
    ("material", "layer"): "layers",
    ("composite", "composite_joint"): "compositeJoints",
    ("composite", "composite_skin"): "compositeSkins",
    ("composite", "composite_geom"): "compositeGeoms",
    ("composite", "composite_site"): "compositeSites",
    ("flexcomp", "flexcomp_edge"): "flexcompEdges",
    ("flexcomp", "elasticity"): "flexElasticities",
    ("flexcomp", "flexcomp_contact"): "flexContacts",
    ("flexcomp", "pin"): "flexcompPins",
    ("deformable", "flex"): "flexs",
    ("deformable", "skin"): "skins",
    ("flex", "flexcomp_contact"): "flexContacts",
    ("flex", "flex_edge"): "flexEdges",
    ("flex", "elasticity"): "flexElasticities",
    ("contact", "pair"): "pairs",
    ("contact", "exclude"): "excludes",
    ("fixed", "fixed_joint"): "fixedJoints",
    ("custom", "numeric"): "numerics",
    ("custom", "text"): "texts",
    ("custom", "tuple"): "tuples",
    ("tuple", "element"): "tupleElements",
    ("keyframe", "key"): "keys",
}

# XML tags routing a child list where the tag is not the child element's own.
# `<mujoco>`'s body slot is written `<worldbody>`; the schema states this only
# through the `worldbody` alias row, which carries no tag binding of its own.
CHILD_TAGS = {
    ("mujoco", "worldbody"): "worldbody",
}


# --------------------------------------------------------------------------- #
# Reference namespaces spanning several elements                               #
# --------------------------------------------------------------------------- #
# A `ref<ns>` is typed by the element declaring `id<ns>`. Where several elements
# declare into one namespace, the reference is typed by a union of them: the
# interleave unions above already name four of these exactly, and these two
# namespaces need a union that is not also a child list. The schema determines
# union membership; only the public C++ union names belong here.
REF_UNIONS = {
    "joint": "JointAny",
    "flex": "FlexAny",
}

# Dynamic object references are derived by the frontend from upstream pairs:
# string objname + objtype, and string refname + reftype. No per-element list is
# needed; an added frame sensor inherits the same target tracking automatically.

# Attributes carrying an angle, converted degree->radian at IO under
# `compiler angle="degree"`. The schema types them as plain doubles.
ANGLE_ATTRS = {
    ("joint", "range"),
    ("joint", "ref"),
    ("joint", "springref"),
    ("replicate", "euler"),
}

# The upstream schema does not annotate angular units. The angle-name/doc
# addition gate therefore requires an explicit answer when it encounters a
# newly named angle candidate. This table records false positives only.
NOT_ANGLE: dict[tuple[str, str], str] = {}


# --------------------------------------------------------------------------- #
# Schema orientation variants                                                 #
# --------------------------------------------------------------------------- #
# The schema declares alternative orientation spellings as one variant group.
# This name describes the import fold performed by the UE XML gateway through
# MuJoCo's public orientation resolver. Ordinary `reading=custom` and
# `writing=custom` facets belong to native MuJoCo, not an overlay handler table.
VARIANT_GROUP_RESOLVERS = {
    "orientation": "orientation",
}
