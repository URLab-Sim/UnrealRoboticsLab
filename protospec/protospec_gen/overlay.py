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
    ("mujoco", "equality"): "equalitys",
    ("mujoco", "actuator"): "actuators",
    ("mujoco", "sensor"): "sensors",
    ("mujoco", "custom"): "customs",
    ("mujoco", "keyframe"): "keyframes",
    ("compiler", "lengthrange"): "lengthRanges",
    ("option", "flag"): "flags",
    ("visual", "global"): "visualGlobals",
    ("visual", "quality"): "visualQualitys",
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
    ("asset", "mesh"): "meshs",
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
    ("flexcomp", "elasticity"): "flexElasticitys",
    ("flexcomp", "flexcomp_contact"): "flexContacts",
    ("flexcomp", "pin"): "flexcompPins",
    ("deformable", "flex"): "flexs",
    ("deformable", "skin"): "skins",
    ("flex", "flexcomp_contact"): "flexContacts",
    ("flex", "flex_edge"): "flexEdges",
    ("flex", "elasticity"): "flexElasticitys",
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


# --------------------------------------------------------------------------- #
# Unreal metadata                                                              #
# --------------------------------------------------------------------------- #
# What KIND of quantity each three- and four-double attribute carries, which
# decides the reflected type the UE profile stores it in.
#
# The schema cannot answer this. It types everything as `double[N]` and carries
# no unit or handedness facet, so `pos` (a point, metres, right-handed),
# `axis` (a direction, dimensionless, right-handed), `rgb1` (a colour) and
# `diaginertia` (kg m^2) are one declaration to it. They are not one thing to
# Unreal: crossing into UE space negates Y and scales by 100 for a point,
# negates Y alone for a direction, and does nothing at all for the other two.
# A single C++ type therefore cannot carry the rule -- `FVector` would span all
# four -- so the emitter gives each kind its own reflected type and the wrong
# conversion stops being expressible. Conversions live once, in those types
# (MuJoCo/Spec/MjFrameTypes.h); this table only says which kind an attribute is.
#
# The kinds, and why each converts the way it does:
#
#   position     a point in MuJoCo's frame. -> FMjPosition3 (flip Y, x100)
#   direction    an axis or ray. -> FMjDirection3 (flip Y, no scale: scaling a
#                direction would be meaningless)
#   physical     a physical vector carrying an SI magnitude (m/s^2, m/s, tesla).
#                Crossing rule identical to `direction` -- flip Y, leave the
#                magnitude in its SI unit -- so it shares FMjDirection3; the kind
#                is spelled separately because the two are not interchangeable to
#                a reader, only to the conversion.
#   orientation  the canonical MJCF quat, [w, x, y, z]. -> FMjQuatRot
#   angle        degrees or radians per `compiler angle=` (`euler`), or an axis
#                and an angle packed together (`axisangle`).
#   length       metres, but never a UE transform: <hfield size> is asset extent
#                and <flexcomp spacing> is a generator parameter. Geom and site
#                `size`, the one length that does reach a UE transform, is a
#                range arity and gets its own seam.
#   ratio        dimensionless multiplier.
#   colour       RGB or RGBA.
#   index        a grid count or cell index.
#   inertia      a principal-moment triple, kg m^2.
#   param        packed actuator gain/bias parameters, unit per transmission.
#
# Everything from `angle` down converts to nothing, and that is the point: for
# those quantities no conversion is the only conversion that is never wrong, so
# they store as FMjVec3 (three doubles, no ToUnreal) or, at four components, as
# a plain array.
#
# Keyed by attribute name, as the shapes are: an attribute called `pos` is a
# position wherever it appears. The gate is EXHAUSTIVE -- every attribute the
# schema declares anywhere as a fixed `double[3]` or `double[4]` must appear
# here exactly once, and every name here must still be declared at that shape.
# A new spatial attribute therefore fails generation by name instead of
# defaulting into some plausible-looking storage.
#
# Fixed `float` arities are deliberately out of scope: `float[4]` is the RGBA
# storage the emitter maps to FLinearColor and `float[3]` is the light and
# headlight colour/coefficient triples. Neither is ever a frame quantity, and
# neither has a conversion to get wrong.
UE_KIND = {
    # -- position ---------------------------------------------------------- #
    "pos": "position",        # body / geom / site / camera / light / frame /
                              # flexcomp / inertial / joint
    "anchor": "position",     # connect / weld
    "offset": "position",     # composite / replicate
    "center": "position",     # statistic
    "refpos": "position",     # mesh
    "bindpos": "position",    # skin bone
    "origin": "position",     # flexcomp: the 2D-to-3D mesh generation origin
    # -- direction --------------------------------------------------------- #
    "axis": "direction",      # joint / composite joint
    "dir": "direction",       # light
    "zaxis": "direction",     # flexcomp (its orientation spellings are plain
                              # attributes, not the schema's variant group)
    # -- physical vector --------------------------------------------------- #
    "gravity": "physical",    # option, m/s^2
    "wind": "physical",       # option, m/s
    "magnetic": "physical",   # option, tesla
    # -- orientation ------------------------------------------------------- #
    "quat": "orientation",    # body / geom / site / camera / frame / composite /
                              # flexcomp / inertial
    "refquat": "orientation",  # mesh
    "bindquat": "orientation",  # skin bone
    # -- no conversion ----------------------------------------------------- #
    "euler": "angle",         # flexcomp / replicate
    "axisangle": "angle",     # flexcomp: three direction components and an angle
    "spacing": "length",      # flexcomp
    "size": "length",         # hfield: x radius, y radius, z top, z bottom
    "scale": "ratio",         # mesh / flexcomp
    "rgb1": "colour",         # texture
    "rgb2": "colour",         # texture
    "markrgb": "colour",      # texture
    "rgba": "colour",         # composite geom / site / skin, flexcomp
    "count": "index",         # flexcomp
    "cellcount": "index",     # flexcomp
    "cell": "index",          # flexstrain
    "diaginertia": "inertia",  # inertial
    "bias": "param",          # cylinder actuator: biasprm[0..2]
}

# Attributes the details panel folds into Unreal's "Advanced" disclosure.
#
# An MJCF element is not a user interface. `geom` declares thirty-odd
# attributes and a person placing a collision shape is looking at maybe six of
# them; the rest are solver tuning they will touch once, if ever, and reading
# past them every time is the cost of showing them at the same rank as `size`.
# Unreal already has the answer -- AdvancedDisplay puts a property behind a
# disclosure arrow, present and editable, just not first.
#
# What belongs here is only ever "rarely authored", never "unimportant": the
# constraint solver's own parameters, which MuJoCo's defaults are tuned for and
# which mean nothing without the solver documentation open; the `user` payload,
# which is an application's private array; and the plugin and rendering-hint
# tails. Anything that changes what the element IS -- a type, a size, a pose, a
# reference, a range -- stays in front, whatever its arity.
#
# Keyed by attribute name across every element, as UE_KIND is: `solref` means
# the same thing and warrants the same rank wherever it appears. There is no
# exhaustiveness gate here and there should not be, because unlike a storage
# kind there is no wrong answer a missing entry produces -- an attribute nobody
# listed is simply shown, which is the safe default. The gate that does apply
# is the ordinary one: a name here that the schema no longer declares anywhere
# is an error.
# --------------------------------------------------------------------------- #
# Element families                                                             #
# --------------------------------------------------------------------------- #
# The subfolder each generated element header lands in. One flat directory of
# 150-odd headers tells a reader nothing; the schema already groups elements by
# the section they are declared under, so that grouping is the one used.
#
# UE_FAMILY_OF_SECTION keys are the top-level children of `mujoco`. An element
# declared under two sections -- `geom` under both `body` and `default`, every
# actuator under both `actuator` and `default` -- takes the non-`default` one,
# because `<default>` mirrors the whole grammar and would otherwise swallow it.
#
# UE_FAMILY_OF_ELEMENT overrides that per element. It exists for `body`, whose
# children are too varied for one folder and which the old hand-written tree
# split the same way, and for the handful of elements whose section is not where
# a reader would look for them.
UE_FAMILY_OF_SECTION = {
    "asset": "Assets",
    "deformable": "Deformable",
    "contact": "Constraints",
    "equality": "Constraints",
    "tendon": "Tendons",
    "actuator": "Actuators",
    "sensor": "Sensors",
    "custom": "Custom",
    "keyframe": "Keyframes",
    "default": "Defaults",
    "extension": "Extensions",
    "compiler": "Options",
    "option": "Options",
    "size": "Options",
    "statistic": "Options",
    "visual": "Options",
    "body": "Bodies",
}

UE_FAMILY_OF_ELEMENT = {
    # <body>'s children, split as the old tree split them.
    "geom": "Geometry",
    "site": "Geometry",
    "joint": "Joints",
    "freejoint": "Joints",
    "composite": "Deformable",
    "flexcomp": "Deformable",
    # Declared under <body>, but they are the scene's optics rather than its
    # structure, and a reader looks for them by name rather than by parent.
    "camera": "Cameras",
    "light": "Cameras",
    # The root element is the spec itself and belongs above the families.
    "mujoco": "",
}

UE_ADVANCED = {
    # -- constraint solver ------------------------------------------------- #
    "solref",
    "solimp",
    "solreffriction",
    "solimpfriction",
    "solmix",
    "solreflimit",
    "solimplimit",
    "solreffix",
    "solimpfix",
    "margin",
    "gap",
    # -- application payload ------------------------------------------------ #
    "user",
    # -- fluid model -------------------------------------------------------- #
    "fluidshape",
    "fluidcoef",
}

# Attributes whose PascalCase spelling would collide with something the
# component hierarchy already declares. `active` is the whole list: its accessor
# would be `SetActive`, which UActorComponent declares as a virtual UFUNCTION
# with a different signature, so UHT would reject the class. The MJCF spelling is
# unaffected -- only the C++ member and its accessors are renamed. The emitter
# fails on any collision not listed here rather than emitting a class that will
# not build.
UE_MEMBER_NAMES = {
    "active": "ActiveFlag",
}


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


# --------------------------------------------------------------------------- #
# Spec-write bindings                                                          #
# --------------------------------------------------------------------------- #
# How each element reaches an mjSpec: one category per element, drift-gated
# against the schema so an element the schema gains or drops fails generation
# rather than silently losing its write.
#
#   body_scoped(fn)   created on the owning body by `fn`
#   spec_scoped(fn)   created on the spec by `fn`
#   parent_embedded   applies its fields onto a struct its PARENT owns, and
#                     creates nothing
#   spec_embedded     applies its fields onto a struct mjSpec owns
#   hook              hand code owns the write; the value names the hook
#   section           a grouping tag: creates nothing, binds no struct, and is
#                     recursed into in authored order
#
# The macro sub-elements are `hook` by association with the macro bridge: a
# macro hands its whole subtree over before per-element dispatch sees any of its
# children, so they are never created individually, and reaching one outside a
# macro subtree is a diagnostic rather than a construction.
SPEC_CREATE = {
    "accelerometer": ("spec_scoped", "mjs_addSensor"),
    "actuator": ("section", None),
    "actuator_plugin": ("hook", "plugins"),
    "actuatorfrc": ("spec_scoped", "mjs_addSensor"),
    "actuatorpos": ("spec_scoped", "mjs_addSensor"),
    "actuatorvel": ("spec_scoped", "mjs_addSensor"),
    "adhesion": ("hook", "actuator_shorthand"),
    "asset": ("section", None),
    "attach": ("hook", "nested_model"),
    "ballangvel": ("spec_scoped", "mjs_addSensor"),
    "ballquat": ("spec_scoped", "mjs_addSensor"),
    "body": ("body_scoped", "mjs_addBody"),
    "bone": ("hook", "skin_bones"),
    "camera": ("body_scoped", "mjs_addCamera"),
    "camprojection": ("spec_scoped", "mjs_addSensor"),
    "clock": ("spec_scoped", "mjs_addSensor"),
    "compiler": ("spec_embedded", None),
    "composite": ("hook", "macro_bridge"),
    "composite_geom": ("hook", "macro_bridge"),
    "composite_joint": ("hook", "macro_bridge"),
    "composite_site": ("hook", "macro_bridge"),
    "composite_skin": ("hook", "macro_bridge"),
    "config": ("hook", "plugins"),
    "connect": ("spec_scoped", "mjs_addEquality"),
    "contact": ("section", None),
    "custom": ("section", None),
    "cylinder": ("hook", "actuator_shorthand"),
    "damper": ("hook", "actuator_shorthand"),
    "dcmotor": ("hook", "actuator_shorthand"),
    "default": ("spec_scoped", "mjs_addDefault"),
    "default_equality": ("parent_embedded", None),
    "default_tendon": ("parent_embedded", None),
    "deformable": ("section", None),
    "distance": ("spec_scoped", "mjs_addSensor"),
    "e_kinetic": ("spec_scoped", "mjs_addSensor"),
    "e_potential": ("spec_scoped", "mjs_addSensor"),
    "elasticity": ("parent_embedded", None),
    "element": ("hook", "tuple_elements"),
    "equality": ("section", None),
    "equality_flex": ("spec_scoped", "mjs_addEquality"),
    "equality_joint": ("spec_scoped", "mjs_addEquality"),
    "equality_tendon": ("spec_scoped", "mjs_addEquality"),
    "exclude": ("spec_scoped", "mjs_addExclude"),
    "extension": ("hook", "plugins"),
    "extension_plugin": ("hook", "plugins"),
    "fixed": ("spec_scoped", "mjs_addTendon"),
    "fixed_joint": ("hook", "tendon_path"),
    "flag": ("hook", "option_flags"),
    "flex": ("spec_scoped", "mjs_addFlex"),
    "flex_edge": ("parent_embedded", None),
    "flexcomp": ("hook", "macro_bridge"),
    "flexcomp_contact": ("parent_embedded", None),
    "flexcomp_edge": ("hook", "macro_bridge"),
    "flexstrain": ("spec_scoped", "mjs_addEquality"),
    "flexvert": ("spec_scoped", "mjs_addEquality"),
    "force": ("spec_scoped", "mjs_addSensor"),
    "frame": ("body_scoped", "mjs_addFrame"),
    "frameangacc": ("spec_scoped", "mjs_addSensor"),
    "frameangvel": ("spec_scoped", "mjs_addSensor"),
    "framelinacc": ("spec_scoped", "mjs_addSensor"),
    "framelinvel": ("spec_scoped", "mjs_addSensor"),
    "framepos": ("spec_scoped", "mjs_addSensor"),
    "framequat": ("spec_scoped", "mjs_addSensor"),
    "framexaxis": ("spec_scoped", "mjs_addSensor"),
    "frameyaxis": ("spec_scoped", "mjs_addSensor"),
    "framezaxis": ("spec_scoped", "mjs_addSensor"),
    "freejoint": ("body_scoped", "mjs_addFreeJoint"),
    "fromto": ("spec_scoped", "mjs_addSensor"),
    "general": ("hook", "actuator_shorthand"),
    "geom": ("body_scoped", "mjs_addGeom"),
    "global": ("spec_embedded", None),
    "gyro": ("spec_scoped", "mjs_addSensor"),
    "headlight": ("spec_embedded", None),
    "hfield": ("spec_scoped", "mjs_addHField"),
    "inertial": ("parent_embedded", None),
    "insidesite": ("spec_scoped", "mjs_addSensor"),
    "instance": ("hook", "plugins"),
    "intvelocity": ("hook", "actuator_shorthand"),
    "joint": ("body_scoped", "mjs_addJoint"),
    "jointactuatorfrc": ("spec_scoped", "mjs_addSensor"),
    "jointlimitfrc": ("spec_scoped", "mjs_addSensor"),
    "jointlimitpos": ("spec_scoped", "mjs_addSensor"),
    "jointlimitvel": ("spec_scoped", "mjs_addSensor"),
    "jointpos": ("spec_scoped", "mjs_addSensor"),
    "jointvel": ("spec_scoped", "mjs_addSensor"),
    "key": ("spec_scoped", "mjs_addKey"),
    "keyframe": ("section", None),
    "layer": ("hook", "material_layers"),
    "lengthrange": ("parent_embedded", None),
    "light": ("body_scoped", "mjs_addLight"),
    "magnetometer": ("spec_scoped", "mjs_addSensor"),
    "map": ("spec_embedded", None),
    "material": ("spec_scoped", "mjs_addMaterial"),
    "mesh": ("spec_scoped", "mjs_addMesh"),
    "model": ("hook", "nested_model"),
    "motor": ("hook", "actuator_shorthand"),
    "mujoco": ("section", None),
    "muscle": ("hook", "actuator_shorthand"),
    "normal": ("spec_scoped", "mjs_addSensor"),
    "numeric": ("spec_scoped", "mjs_addNumeric"),
    "option": ("spec_embedded", None),
    "orientation": ("hook", "actuator_shorthand"),
    "pair": ("spec_scoped", "mjs_addPair"),
    "pid": ("hook", "actuator_shorthand"),
    "pin": ("hook", "macro_bridge"),
    "plugin": ("hook", "plugins"),
    "position": ("hook", "actuator_shorthand"),
    "pulley": ("hook", "tendon_path"),
    "quality": ("spec_embedded", None),
    "rangefinder": ("spec_scoped", "mjs_addSensor"),
    "replicate": ("hook", "macro_bridge"),
    "rgba": ("spec_embedded", None),
    "scale": ("spec_embedded", None),
    "sensor": ("section", None),
    "sensor_contact": ("spec_scoped", "mjs_addSensor"),
    "sensor_plugin": ("hook", "plugins"),
    "site": ("body_scoped", "mjs_addSite"),
    "size": ("spec_embedded", None),
    "skin": ("spec_scoped", "mjs_addSkin"),
    "spatial": ("spec_scoped", "mjs_addTendon"),
    "spatial_geom": ("hook", "tendon_path"),
    "spatial_site": ("hook", "tendon_path"),
    "statistic": ("spec_embedded", None),
    "subtreeangmom": ("spec_scoped", "mjs_addSensor"),
    "subtreecom": ("spec_scoped", "mjs_addSensor"),
    "subtreelinvel": ("spec_scoped", "mjs_addSensor"),
    "tactile": ("spec_scoped", "mjs_addSensor"),
    "tendon": ("section", None),
    "tendonactuatorfrc": ("spec_scoped", "mjs_addSensor"),
    "tendonlimitfrc": ("spec_scoped", "mjs_addSensor"),
    "tendonlimitpos": ("spec_scoped", "mjs_addSensor"),
    "tendonlimitvel": ("spec_scoped", "mjs_addSensor"),
    "tendonpos": ("spec_scoped", "mjs_addSensor"),
    "tendonvel": ("spec_scoped", "mjs_addSensor"),
    "text": ("spec_scoped", "mjs_addText"),
    "texture": ("spec_scoped", "mjs_addTexture"),
    "torque": ("spec_scoped", "mjs_addSensor"),
    "touch": ("spec_scoped", "mjs_addSensor"),
    "tuple": ("spec_scoped", "mjs_addTuple"),
    "user": ("spec_scoped", "mjs_addSensor"),
    "velocimeter": ("spec_scoped", "mjs_addSensor"),
    "velocity": ("hook", "actuator_shorthand"),
    "visual": ("spec_embedded", None),
    "weld": ("spec_scoped", "mjs_addEquality"),
}

# Where an embedded element's struct LIVES. The schema names the struct an
# element binds, but not how to reach one that no `mjs_add*` call returns, and
# the spellings are not derivable: mjStatistic is reached as `spec->stat`, and
# `lengthrange` targets its parent's `LRopt` rather than the parent's own
# struct.
#
# One row per element in category `spec_embedded` or `parent_embedded`, except
# the visual sub-blocks, whose `field=` facet already names them relative to
# their parent. The value is (parent struct C type or None, target expression);
# the expression is C++ over `Spec` for a spec-embedded element and over
# `Parent` for a parent-embedded one.
SPEC_TARGET = {
    "compiler": (None, "Spec->compiler"),
    "option": (None, "Spec->option"),
    "size": (None, "*Spec"),
    "statistic": (None, "Spec->stat"),
    # `visual` binds no struct of its own; the row is how its sub-blocks are
    # reached, and it is what makes the gate's "one row per element" hold.
    "visual": (None, "Spec->visual"),

    "inertial": ("mjsBody", "*Parent"),
    "elasticity": ("mjsFlex", "*Parent"),
    "flex_edge": ("mjsFlex", "*Parent"),
    "flexcomp_contact": ("mjsFlex", "*Parent"),
    "lengthrange": ("mjsCompiler", "Parent->LRopt"),
    # The class partials hang off the owning default's member structs, which
    # mjsDefault holds by pointer.
    "default_equality": ("mjsDefault", "*Parent->equality"),
    "default_tendon": ("mjsDefault", "*Parent->tendon"),
}

# The hook that owns a write the generated path cannot express. Keyed by element
# name for a whole-element hook, or by (element, attribute) where the generated
# code applies the plain fields and the hook owns only the named ones.
#
# `lib/io` registers every name that appears here, the same completeness rule
# READ_HANDLERS carries.
SPEC_WRITE_HANDLERS = {
    # Transmission: exactly one of joint/tendon/site/body and friends elects
    # target and trntype together, so no one of them can be written alone.
    # Keyed on the target attribute wherever a spelling carries no
    # `cranklength`: the hook is attached per attribute, so a spelling whose
    # only transmission attributes are targets has to name one of them or it
    # never elects a target at all.
    ("general", "body"): "transmission",
    ("orientation", "site"): "transmission",
    ("adhesion", "body"): "transmission",
    ("general", "cranklength"): "transmission",
    ("motor", "cranklength"): "transmission",
    ("position", "cranklength"): "transmission",
    ("velocity", "cranklength"): "transmission",
    ("intvelocity", "cranklength"): "transmission",
    ("pid", "cranklength"): "transmission",
    ("damper", "cranklength"): "transmission",
    ("cylinder", "cranklength"): "transmission",
    ("muscle", "cranklength"): "transmission",
    ("dcmotor", "cranklength"): "transmission",
    ("actuator_plugin", "cranklength"): "transmission",

    # The actuator shorthands' own parameters: `mjs_setTo*` derives gain, bias
    # and dyntype together from them, and inherits from what is already set.
    ("general", "input"): "actuator_shorthand",

    # Single-attribute folds: the alias has no field of its own, so the write is
    # the canonical field's, and the alias is the reader's spelling of it.
    ("inertial", "axisangle"): "input_fold",
    ("inertial", "euler"): "input_fold",
    ("inertial", "xyaxes"): "input_fold",
    ("inertial", "zaxis"): "input_fold",
    ("inertial", "fullinertia"): "input_fold",
    ("light", "directional"): "input_fold",
    ("cylinder", "diameter"): "input_fold",

    # The orientation variant group, on every element that carries it. The
    # schema declares it `group orientation variant`, so the members are the
    # alternative spellings of the group's first member and fold into it by the
    # same rule the aliases above follow.
    ("body", "axisangle"): "input_fold",
    ("body", "euler"): "input_fold",
    ("body", "xyaxes"): "input_fold",
    ("body", "zaxis"): "input_fold",
    ("camera", "axisangle"): "input_fold",
    ("camera", "euler"): "input_fold",
    ("camera", "xyaxes"): "input_fold",
    ("camera", "zaxis"): "input_fold",
    ("frame", "axisangle"): "input_fold",
    ("frame", "euler"): "input_fold",
    ("frame", "xyaxes"): "input_fold",
    ("frame", "zaxis"): "input_fold",
    ("geom", "axisangle"): "input_fold",
    ("geom", "euler"): "input_fold",
    ("geom", "xyaxes"): "input_fold",
    ("geom", "zaxis"): "input_fold",
    ("site", "axisangle"): "input_fold",
    ("site", "euler"): "input_fold",
    ("site", "xyaxes"): "input_fold",
    ("site", "zaxis"): "input_fold",

    # A material's `texture=` is the RGB entry of its layer list.
    ("material", "texture"): "material_layers",

    # The authored suffix form parses into a byte count on mjSpec.
    ("size", "memory"): "size_memory",

    # Authored group ids fold into mjOption's disableactuator bitmask, one bit
    # per id, with the engine's non-negative and <= 30 checks. There is no
    # vector on the option struct to write plainly, so the fold is the write.
    ("option", "actuatorgroupdisable"): "option_flags",

    # Below: attributes with no field on the struct their element binds. Each
    # row is justified by an absence the emitter re-checks against mjspec.h on
    # every run, so a row that stops being needed fails generation rather than
    # lingering as hand code the generator could have covered.

    # mjsEquality carries name1/name2/objtype/data and nothing else, so every
    # operand of every subtype is a fold: the subtype elects objtype and packs
    # its operands into the name pair and its numbers into data.
    ("connect", "body1"): "equality_fold",
    ("connect", "body2"): "equality_fold",
    ("connect", "anchor"): "equality_fold",
    ("connect", "site1"): "equality_fold",
    ("connect", "site2"): "equality_fold",
    ("weld", "body1"): "equality_fold",
    ("weld", "body2"): "equality_fold",
    ("weld", "relpose"): "equality_fold",
    ("weld", "anchor"): "equality_fold",
    ("weld", "site1"): "equality_fold",
    ("weld", "site2"): "equality_fold",
    ("weld", "torquescale"): "equality_fold",
    ("equality_joint", "joint1"): "equality_fold",
    ("equality_joint", "joint2"): "equality_fold",
    ("equality_joint", "polycoef"): "equality_fold",
    ("equality_tendon", "tendon1"): "equality_fold",
    ("equality_tendon", "tendon2"): "equality_fold",
    ("equality_tendon", "polycoef"): "equality_fold",
    ("equality_flex", "flex"): "equality_fold",
    ("flexvert", "flex"): "equality_fold",
    ("flexstrain", "flex"): "equality_fold",
    ("flexstrain", "cell"): "equality_fold",

    # The same shape one level over: the irregular sensors name their operands
    # per tag, and mjsSensor stores them in objname/refname/objtype/reftype,
    # with the keyword sets in intprm.
    ("rangefinder", "site"): "sensor_fold",
    ("rangefinder", "camera"): "sensor_fold",
    ("rangefinder", "data"): "sensor_fold",
    ("distance", "geom1"): "sensor_fold",
    ("distance", "geom2"): "sensor_fold",
    ("distance", "body1"): "sensor_fold",
    ("distance", "body2"): "sensor_fold",
    ("normal", "geom1"): "sensor_fold",
    ("normal", "geom2"): "sensor_fold",
    ("normal", "body1"): "sensor_fold",
    ("normal", "body2"): "sensor_fold",
    ("fromto", "geom1"): "sensor_fold",
    ("fromto", "geom2"): "sensor_fold",
    ("fromto", "body1"): "sensor_fold",
    ("fromto", "body2"): "sensor_fold",
    ("sensor_contact", "geom1"): "sensor_fold",
    ("sensor_contact", "geom2"): "sensor_fold",
    ("sensor_contact", "body1"): "sensor_fold",
    ("sensor_contact", "body2"): "sensor_fold",
    ("sensor_contact", "subtree1"): "sensor_fold",
    ("sensor_contact", "subtree2"): "sensor_fold",
    ("sensor_contact", "site"): "sensor_fold",
    ("sensor_contact", "num"): "sensor_fold",
    ("sensor_contact", "data"): "sensor_fold",
    ("sensor_contact", "reduce"): "sensor_fold",
    ("tactile", "geom"): "sensor_fold",
    ("tactile", "mesh"): "sensor_fold",
    ("tactile", "user"): "sensor_fold",
    ("user", "user"): "sensor_fold",
    # A keyword spelled as a plain string, because <user> names the object kind
    # rather than choosing between per-kind attributes the way the rest do.
    ("user", "objtype"): "sensor_fold",

    # A double list spelled as one string, so the parse is the write.
    ("numeric", "data"): "numeric_data",

    # strippath lands on mjSpec rather than on mjsCompiler, coordinate has no
    # field because the engine reads it only to reject the removed global form,
    # and assetdir fans out into meshdir and texturedir.
    ("compiler", "strippath"): "compiler_placement",
    ("compiler", "coordinate"): "compiler_placement",
    ("compiler", "assetdir"): "compiler_placement",

    # Generated geometry parameters and the positional cube-face vector, none of
    # which is a field on the asset's own struct.
    ("mesh", "builtin"): "asset_builtin",
    ("mesh", "params"): "asset_builtin",
    ("hfield", "elevation"): "asset_builtin",
    ("texture", "fileright"): "asset_builtin",
    ("texture", "fileleft"): "asset_builtin",
    ("texture", "fileup"): "asset_builtin",
    ("texture", "filedown"): "asset_builtin",
    ("texture", "filefront"): "asset_builtin",
    ("texture", "fileback"): "asset_builtin",

    # A keyword the engine maps to a dof layout, with no field behind it.
    ("flex", "dof"): "flex_layout",
}

# Attributes the gate asks about whose plain generated write is correct, with
# the reason. Same discipline as READ_NOTES: silence is not allowed, so every
# one of these is a stated decision rather than an omission.
SPEC_WRITE_NOTES = {
    # Canonical targets of the folds above: the stored field is written plainly,
    # and the fold is what put the value there.
    ("inertial", "quat"): "the canonical orientation field, written plainly "
        "onto the parent body's iquat",
    ("inertial", "diaginertia"): "the canonical inertia field, written plainly "
        "onto the parent body's inertia",
    ("light", "type"): "the canonical field the directional spelling folds "
        "into, a plain enum write",
    ("cylinder", "area"): "the canonical field the diameter spelling folds "
        "into, a plain double write",
    ("spatial", "springlength"): "plain double pair; the reader's resolver "
        "exists to accept the one-value spelling, and the stored value is what "
        "the spec takes",
    ("fixed", "springlength"): "as spatial.springlength",
    ("default_tendon", "springlength"): "as spatial.springlength",

    # Values the engine interprets at compile. ProtoSpec stores what was
    # authored and the spec takes the same, so the conversion stays MuJoCo's.
    ("mesh", "maxhullvert"): "plain int; the engine clamps it against the "
        "vertex count at compile",
    ("skin", "group"): "plain int the engine range-checks",
    ("geom", "fluidshape"): "plain keyword the engine stores as a 0/1 flag",
    ("flex", "cellcount"): "plain int list the engine derives counts from",

    # Legacy sizing the engine accepts only to reject: there is nothing to write.
    ("size", "njmax"): "removed legacy sizing; the engine rejects it, so the "
        "spec never carries it",
    ("size", "nconmax"): "as size.njmax",
    ("size", "nstack"): "as size.njmax",

    # The identity attribute is written centrally after creation, by mjs_setName
    # rather than by any element's field application. This is also what settles
    # the keyframe vectors: mjsKey's own vectors are plain unbounded writes, so
    # no keyframe hook is needed.
    ("key", "name"): "the identity attribute, written centrally by mjs_setName "
        "after creation, not through this element's fields",
}
