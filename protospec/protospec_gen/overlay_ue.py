"""ProtoSpec overlay, Unreal half: how the schema presents in Unreal.

The grammar half (:mod:`protospec_gen.overlay`) says what MJCF means and how
ProtoSpec spells it in C++. This file says what Unreal does with it: which
reflected type a spatial attribute stores in, which properties sit behind the
details panel's Advanced disclosure, which folder a generated element header
lands in, the member renames Unreal's own hierarchy forces, and how each element
and attribute reaches an mjSpec.

Nothing here changes what the grammar means, and nothing here is consulted when
the grammar is read or written: a ProtoSpec build without Unreal ignores this
file entirely. It is split out so that a reader of the grammar tables is not
reading Unreal presentation, and vice versa.

The gates are the same gates. :mod:`protospec_gen.frontend` drift-checks these
tables against the schema exactly as it does the grammar half, so an entry
naming something the schema no longer declares is an error here too.
"""

from __future__ import annotations

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

# Attributes that take no tooltip from MuJoCo's reference manual, keyed
# `element.attribute` with the reason. Two kinds of entry, and no others:
# an attribute upstream has genuinely stopped documenting (the drift gate in
# `xmlref.py` names it, and this is where the answer goes), and one whose
# recorded binding a human read and judged to be about a different thing.
#
# A waived attribute keeps the bare `MJCF: <name>` tooltip. Empty is better
# than wrong: a user reading a description of the wrong attribute has no way to
# tell, where a user reading nothing goes and looks it up.
XMLREF_WAIVERS: dict[str, str] = {
    # Both bind to `plugin-instance-name`, which documents the name of a
    # <plugin> instance under <extension>. These two are the actuator's and the
    # sensor's own names, and the manual leaves them to its shared attribute
    # chapter.
    "actuator_plugin.name": "binds to the <extension><plugin><instance> name, "
        "which is a different element's name",
    "sensor_plugin.name": "as actuator_plugin.name",
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
    # Transmission's own attributes, where the gate asks about them.
    # `cranklength` resolves to a field of mjsActuator, so without a row here it
    # would be written twice: once plainly and once by the hook that validates
    # it against the elected trntype. `general.body` the reader interprets, so
    # the gate demands a disposition for it and the hook is the answer. Neither
    # row is what ATTACHES the hook -- that is stated per element, in
    # SPEC_WRITE_ELEMENT_HOOKS, because the election reads all of the operands
    # together and no one attribute owns it.
    ("general", "body"): "transmission",
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

# Hooks whose concern is the element rather than any one of its attributes.
#
# Transmission election is the case that forced this table. Exactly one of
# joint/jointinparent/tendon/cranksite/site/body decides `target` and `trntype`
# together, so the hook has to read them as a set and none of them owns it.
# Keyed per attribute, the hook was attached only to a spelling that happened to
# declare the keyed attribute, and `orientation` and `adhesion` -- which declare
# no `cranklength` -- silently elected no target at all. Every actuator spelling
# is named here so that the guarantee is stated once per element rather than
# inferred from which attributes the element turned out to have.
SPEC_WRITE_ELEMENT_HOOKS = {
    "general": ("transmission",),
    "motor": ("transmission",),
    "position": ("transmission",),
    "velocity": ("transmission",),
    "intvelocity": ("transmission",),
    "orientation": ("transmission",),
    "pid": ("transmission",),
    "damper": ("transmission",),
    "cylinder": ("transmission",),
    "muscle": ("transmission",),
    "adhesion": ("transmission",),
    "dcmotor": ("transmission",),
    "actuator_plugin": ("transmission",),
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
