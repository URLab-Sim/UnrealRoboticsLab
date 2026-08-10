"""ProtoSpec overlay: the facts `src/xml/mjcf.schema` deliberately does not state.

The schema is the single source of truth for the MJCF grammar -- elements,
attributes, types, arities, defaults, enums, namespaces, presence constraints,
and the mjSpec struct/field bindings. It says nothing about how ProtoSpec spells
those things in C++, how a spec-ordered heterogeneous child list is
assembled from per-tag child declarations, or which hand-written IO handler owns
an attribute the engine reads or writes by hand. Those three gaps are this file,
and nothing else belongs here: every entry below is naming, list shaping, or a
handler binding. No entry changes what the grammar means.

The tables are drift-gated. :mod:`protospec_gen.frontend` fails loudly on an
entry naming an element, attribute, or enum the schema no longer declares, and
on a schema `reading=custom` / `writing=custom` facet that neither binds to a
resolver nor carries a waiver. A schema bump therefore surfaces here as a named
error rather than as silently stale output.
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
# MJCF sections whose spec order across different tags is semantic -- the
# ids and data addresses of actuators, sensors, equality constraints and tendons
# all follow interleaved source order, and a spatial tendon's routing IS the
# interleave of its site/geom/pulley path items. The schema declares one `child`
# row per tag, which cannot express that; these rows say which of an element's
# child declarations collapse into a single ordered heterogeneous list, and what
# to call the list and its item type.
#
# Membership is the named child declarations, plus every element the schema
# aliases (`alias=`) to one of them: `frame` and `replicate` carry `alias=body`,
# so they are admitted wherever a body is, which is what MJCF's NameMatch does.
# A child declaration not named here stays its own homogeneous list.
INTERLEAVE = {
    "body": [("subtree", "BodyChildAny",
              ["body", "joint", "freejoint", "geom", "attach", "site", "camera",
               "light", "plugin", "composite", "flexcomp"])],
    "tendon": [("tendons", "TendonAny", ["spatial", "fixed"])],
    "spatial": [("path", "PathItemAny",
                 ["spatial_site", "spatial_geom", "pulley"])],
    "equality": [("equalities", "EqualityAny",
                  ["connect", "weld", "equality_joint", "equality_tendon",
                   "equality_flex", "flexvert", "flexstrain"])],
    "actuator": [("actuators", "ActuatorAny",
                  ["general", "motor", "position", "velocity", "intvelocity",
                   "orientation", "pid", "damper", "cylinder", "muscle",
                   "adhesion", "dcmotor", "actuator_plugin"])],
    "sensor": [("sensors", "SensorAny",
                ["touch", "accelerometer", "velocimeter", "gyro", "force",
                 "torque", "magnetometer", "camprojection", "rangefinder",
                 "jointpos", "jointvel", "tendonpos", "tendonvel",
                 "actuatorpos", "actuatorvel", "actuatorfrc",
                 "jointactuatorfrc", "tendonactuatorfrc", "ballquat",
                 "ballangvel", "jointlimitpos", "jointlimitvel",
                 "jointlimitfrc", "tendonlimitpos", "tendonlimitvel",
                 "tendonlimitfrc", "framepos", "framequat", "framexaxis",
                 "frameyaxis", "framezaxis", "framelinvel", "frameangvel",
                 "framelinacc", "frameangacc", "subtreecom", "subtreelinvel",
                 "subtreeangmom", "insidesite", "distance", "normal", "fromto",
                 "sensor_contact", "e_potential", "e_kinetic", "clock",
                 "tactile", "user", "sensor_plugin"])],
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
# namespaces need a union that is not also a child list.
REF_UNIONS = {
    "joint": ("JointAny", ["joint", "freejoint"]),
    "flex": ("FlexAny", ["flex", "flexcomp"]),
}

# A reference whose target type is not fixed by the schema but named at runtime
# by a sibling attribute (`objname` is whatever `objtype` says). Stored as a
# plain string; recorded so the referrer scan, rename fixup and referential
# validation cover these slots instead of leaving them untracked.
TARGET_FROM = {
    ("framepos", "objname"): "objtype",
    ("framepos", "refname"): "reftype",
    ("framequat", "objname"): "objtype",
    ("framequat", "refname"): "reftype",
    ("framexaxis", "objname"): "objtype",
    ("framexaxis", "refname"): "reftype",
    ("frameyaxis", "objname"): "objtype",
    ("frameyaxis", "refname"): "reftype",
    ("framezaxis", "objname"): "objtype",
    ("framezaxis", "refname"): "reftype",
    ("framelinvel", "objname"): "objtype",
    ("framelinvel", "refname"): "reftype",
    ("frameangvel", "objname"): "objtype",
    ("frameangvel", "refname"): "reftype",
    ("framelinacc", "objname"): "objtype",
    ("frameangacc", "objname"): "objtype",
    ("insidesite", "objname"): "objtype",
    ("sensor_plugin", "objname"): "objtype",
    ("sensor_plugin", "refname"): "reftype",
    ("user", "objname"): "objtype",
    ("element", "objname"): "objtype",
}

# Attributes carrying an angle, converted degree->radian at IO under
# `compiler angle="degree"`. The schema types them as plain doubles.
ANGLE_ATTRS = {
    ("joint", "range"),
    ("joint", "ref"),
    ("joint", "springref"),
    ("replicate", "euler"),
}

# The two waiver tables below answer the front end's addition-side gates
# (`classified_attrs.json` and `_check_classified_attrs`). Those gates cannot
# tell an angle from a length or a reference from a label, so they use a
# deliberately over-eager name and doc heuristic and demand a human answer for
# every attribute the schema gains that trips it. The answer is either the
# classification table (`ANGLE_ATTRS`, `TARGET_FROM`) or a row here saying why
# the heuristic is wrong for this one. Silence is not an option, so a false
# positive costs one line and a real angle attribute cannot arrive as a plain
# double that is then wrong by 57.3x.
#
# Both are keyed (element, attribute) and both are self-retiring: a row naming
# an attribute the schema no longer declares fails generation.

# Angle-shaped by name or doc text, but not an angle.
NOT_ANGLE: dict[tuple[str, str], str] = {}

# Reference-shaped by name or type, but not a dynamic reference needing a
# TARGET_FROM row. A statically typed `ref<ns>` belongs here: the front end
# already phantom-types it from the namespace's declarers, which is stronger
# than a TARGET_FROM row, so the heuristic firing on it is expected.
NOT_TARGET: dict[tuple[str, str], str] = {}


# --------------------------------------------------------------------------- #
# IO handler bindings                                                          #
# --------------------------------------------------------------------------- #
# An attribute whose authored value is folded into a sibling field at parse end
# rather than stored on its own: MJCF's alternative spellings. It is accepted on
# input (so it is not an unknown attribute), never written back, and has no
# field of its own; the named resolver owns the fold. The five-way orientation
# group is not listed here -- the schema declares it as `group orientation
# variant`, and the frontend folds every variant group into its first member,
# which is the same rule stated once.
INPUT_ALIASES = {
    ("inertial", "axisangle"): "quat",
    ("inertial", "xyaxes"): "quat",
    ("inertial", "zaxis"): "quat",
    ("inertial", "euler"): "quat",
    ("inertial", "fullinertia"): "diaginertia",
    ("light", "directional"): "type",
    ("cylinder", "diameter"): "area",
}

# An MJCF attribute canonicalized into a CHILD list rather than a field. A
# material's `texture=` is exactly the RGB entry of its `<layer>` list, so it has
# no field of its own and the reader converts it to `<layer role="rgb">`.
ELEMENT_INPUT_ALIASES = {
    "material": [("texture", "materiallayer")],
}

# The resolver owning each variant group's fold. The group's first member is the
# stored canonical field; the rest become input aliases of it.
VARIANT_GROUP_RESOLVERS = {
    "orientation": "orientation",
}

# The reader's resolver registry: the handler that owns an attribute whose read
# ProtoSpec cannot drive from the typed row alone. The reader suppresses the
# plain attribute read for every attribute bound here and calls the handler
# instead. `lib/io/mjcf_reader.cc` must register every name that appears here or
# in INPUT_ALIASES' resolvers; `ResolverRegistryComplete` asserts exactly that.
READ_HANDLERS = {
    ("inertial", "quat"): "orientation",
    ("inertial", "axisangle"): "orientation",
    ("inertial", "xyaxes"): "orientation",
    ("inertial", "zaxis"): "orientation",
    ("inertial", "euler"): "orientation",
    ("inertial", "diaginertia"): "inertia",
    ("inertial", "fullinertia"): "inertia",
    ("light", "type"): "lighttype",
    ("light", "directional"): "lighttype",
    ("cylinder", "area"): "cylinderarea",
    ("cylinder", "diameter"): "cylinderarea",
    ("spatial", "springlength"): "springlength",
    ("fixed", "springlength"): "springlength",
    ("default_tendon", "springlength"): "springlength",
}

# `reading=custom` facets ProtoSpec does not bind to a resolver, with the reason.
# The engine hand-reads these because of where they land in mjSpec, not because
# the authored text needs interpreting: ProtoSpec stores the authored value
# through its ordinary typed path and lets MuJoCo do the conversion at compile.
# A few are handled by an element-level reader hook rather than a per-attribute
# resolver, and say so.
READ_NOTES = {
    ("compiler", "strippath"): "plain bool; the engine hand-reads it because it "
        "lands on mjSpec rather than mjsCompiler",
    ("compiler", "coordinate"): "plain keyword; the engine reads it only to "
        "reject the removed global form",
    ("compiler", "assetdir"): "plain path; the engine expands it into "
        "meshdir/texturedir at read",
    ("option", "actuatorgroupdisable"): "plain int list; the engine folds it "
        "into a disable bitmask at read",
    ("size", "memory"): "the reader's Size hook parses the size suffix into a "
        "byte count (MemoryFixup)",
    ("size", "njmax"): "plain int; the engine reads it only to reject the "
        "removed legacy sizing",
    ("size", "nconmax"): "as size.njmax",
    ("size", "nstack"): "as size.njmax",
    ("mesh", "maxhullvert"): "plain int stored verbatim; the engine clamps it "
        "against the vertex count at compile",
    ("mesh", "builtin"): "plain keyword; the engine generates geometry from it "
        "at compile rather than storing it",
    ("mesh", "params"): "plain double list feeding the builtin generator",
    ("hfield", "elevation"): "plain double list; the engine flips rows and "
        "zero-fills to nrow*ncol at compile",
    ("skin", "group"): "plain int; the engine range-checks it at read",
    ("geom", "fluidshape"): "plain keyword; the engine stores it as a 0/1 flag",
    ("flex", "cellcount"): "plain int list; the engine derives counts from it",
    ("flex", "dof"): "plain keyword; the engine maps it to a dof layout",
    ("rangefinder", "data"): "keyword set read through the generic space-"
        "separated path and sorted into enum order",
    ("sensor_contact", "data"): "as rangefinder.data",
    ("key", "name"): "plain string; the engine hand-reads it to index keyframes",
    ("general", "body"): "plain adhesion-style transmission ref",
    ("general", "input"): "keyword set; the engine folds it into an "
        "mjtCtrlInput bitmask at compile",
    ("general", "cranklength"): "plain double; the engine hand-reads it as part "
        "of transmission selection",
    ("motor", "cranklength"): "as general.cranklength",
    ("position", "cranklength"): "as general.cranklength",
    ("velocity", "cranklength"): "as general.cranklength",
    ("intvelocity", "cranklength"): "as general.cranklength",
    ("pid", "cranklength"): "as general.cranklength",
    ("damper", "cranklength"): "as general.cranklength",
    ("cylinder", "cranklength"): "as general.cranklength",
    ("muscle", "cranklength"): "as general.cranklength",
    ("dcmotor", "cranklength"): "as general.cranklength",
    ("actuator_plugin", "cranklength"): "as general.cranklength",
}

# `writing=custom` facets. The engine's writer elects at save time what to emit
# -- a size whose length depends on the geom type, mass or density but not both,
# a fromto the compiler already turned into pos/quat/size. ProtoSpec writes
# exactly the authored fields and re-derives nothing, so none of these needs a
# writer handler; each entry says why.
WRITE_NOTES = {
    ("joint", "pos"): "authored value written verbatim",
    ("joint", "axis"): "authored value written verbatim",
    ("joint", "springdamper"): "compile directive, written verbatim when authored",
    ("joint", "limited"): "authored keyword written verbatim",
    ("joint", "actuatorfrclimited"): "authored keyword written verbatim",
    ("geom", "size"): "authored length written verbatim, not re-derived by type",
    ("geom", "mass"): "authored value written verbatim; no mass/density election",
    ("geom", "density"): "authored value written verbatim",
    ("geom", "shellinertia"): "authored bool written verbatim",
    ("geom", "fromto"): "compile directive, written verbatim when authored",
    ("geom", "pos"): "authored value written verbatim, not mesh-corrected",
    ("geom", "fitscale"): "compile directive, written verbatim when authored",
    ("site", "size"): "authored length written verbatim",
    ("site", "fromto"): "compile directive, written verbatim when authored",
    ("camera", "fovy"): "authored value written verbatim; the intrinsics are "
        "exclusive by constraint, not by writer election",
    ("camera", "focal"): "as camera.fovy",
    ("camera", "focalpixel"): "as camera.fovy",
    ("camera", "principal"): "as camera.fovy",
    ("camera", "principalpixel"): "as camera.fovy",
    ("camera", "sensorsize"): "as camera.fovy",
    ("general", "actdim"): "authored value written verbatim; the engine derives "
        "it from dyntype when saving",
    ("general", "gaintype"): "authored keyword written verbatim",
    ("general", "biastype"): "authored keyword written verbatim",
    ("general", "gainprm"): "authored values written verbatim",
    ("general", "biasprm"): "authored values written verbatim",
}
