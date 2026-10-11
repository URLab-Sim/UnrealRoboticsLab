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
# position wherever it appears. New fixed double[3]/double[4] names use raw
# numeric storage until a UE coordinate or editor policy is deliberately
# assigned. Existing coordinate classifications are validated against the
# upstream shape so they cannot silently lose their conversion.
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
