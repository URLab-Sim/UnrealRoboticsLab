# MJCF Support

What URLab reads from MJCF and writes back. The short answer is all of it:
the model layer is generated from MuJoCo's own schema, so coverage is
whatever the pinned MuJoCo declares. This page is about the places where
that answer needs qualifying, and about how the claim is checked.

Checked against MuJoCo 3.11.1 (`mjVERSION_HEADER 3011001`).

## Coverage is the schema

There is no hand-maintained list of supported elements and no
hand-maintained list of supported attributes. Every element MuJoCo's
`mjcf.schema` declares becomes a component class, and every attribute
becomes a property on it. The current tree has 144 element classes and 48
enums.

| Family | Elements |
|---|---|
| Model blocks | `mujoco`, `compiler`, `lengthrange`, `option`, `flag`, `size`, `statistic`, `visual` and its six sub-blocks |
| Bodies | `body`, `inertial`, `frame`, `attach`, `replicate` |
| Geometry | `geom`, `site` |
| Joints | `joint`, `freejoint` |
| Cameras and lights | `camera`, `light` |
| Assets | `mesh`, `hfield`, `skin`, `bone`, `texture`, `material`, `layer`, `model` |
| Defaults | `default` and every class partial |
| Contact, equality, tendon | `pair`, `exclude`, all seven equality types, spatial and fixed tendons with all four wrap types |
| Actuators | all twelve spellings plus `plugin` |
| Sensors | all 49 sensor tags |
| Custom, keyframe, extension | `numeric`, `text`, `tuple`, `key`, `plugin`, `config` |
| Deformable and macros | `flex`, `flexcomp`, `composite` and their sub-elements |

An attribute upstream adds is not a silent omission. It is a generation
error naming the attribute, until an overlay row says what it means. See
[Generation](generation.md) for the gates that make that true.

## How the claim is checked

Three independent nets, because "the schema declares it" and "the value
survives" are different claims.

**The corpus net** round-trips every model in MuJoCo's own test corpus
through URLab's MJCF reader and writer, loads the written text with
`mj_loadXML`, and diffs the resulting `mjModel` against a stock load of
the original file, field by field: every size, the name table, every
pointer array. Its recorded allowed-failure list is currently empty. Run
it with `protospec/corpus_net.ps1` or `corpus_net.sh`.

**Live compile parity** compiles every fixture in
`Content/TestData/parity/` twice, once through the component tree and
once through `mj_loadXML` of the same file, and diffs the two `mjModel`s
at zero tolerance. This is the one that covers the path an actual
simulation takes, since a simulation never goes through text.

**Compiled-model goldens** in `Content/TestData/goldens/` pin the output
across changes that are not supposed to move it.

## Macros are expanded by MuJoCo, not by URLab

`<replicate>`, `<composite>` and `<flexcomp>` are not elements. They are
compile-time macros that MuJoCo expands during its own read, and there is
no spec-API call for any of them. URLab serializes the macro's subtree
back to MJCF, hands it to MuJoCo's reader, and attaches the expansion, so
the result is exactly what MuJoCo would have produced.

The consequence to know is about addressing, not fidelity. The bodies,
geoms and joints a macro produces exist in the compiled model and not in
the component tree, so there is no component to select or bind for any of
them. `FMjBinding::Find(ObjType, Glob)` is how you reach them by name
pattern.

## What the compiler removes

`<compiler discardvisual>` and `<compiler fusestatic>` both remove
elements during compilation. A component whose element did not survive
reports no id and stays as authoring data, because an id into a table
that no longer holds the element is worse than no id at all.

## Includes are a security boundary

`<include>` is resolved when the document is read. An include whose
resolved path escapes the root model's directory tree is refused by
default: the reader opens untrusted MJCF inside a GUI host, and an
unbounded include is exfiltration-shaped. `FMjDocParseOptions` carries
the opt-in.

## What the viewport draws

Coverage of the format and coverage of the *picture* are different
questions, and the second one is presentation only. The compiled model is
the same either way.

A geom previews as an engine primitive where one fits: plane, sphere,
capsule (a cylinder plus two sphere caps), ellipsoid, cylinder and box. A
`mesh` geom carries its picture as a child mesh component instead. A
`hfield` or `sdf` geom has no preview, which is the honest answer rather
than a wrong one. A geom that resolves to a shape with nothing to draw
records it in `PreviewProblems` on the component, because the viewport is
the only report that case otherwise gets.

## Writing back out

Export is not a re-serialisation of what was imported. An element nobody
touched writes back byte for byte, because the element remembers which
Unreal asset its `file` stands for and a swap is a comparison rather than
a guess.

When an asset was swapped, it is written back as a file so the spec stays
portable: a `<mesh>` as OBJ, a `<texture>` as PNG, both into a
`urlab_assets` folder inside the directory MJCF already resolves that
element's `file` against. `<hfield>` and `<skin>` name files but have no
imported Unreal asset to write back, so they are deliberately left alone.

Only authored attributes are written. An attribute you never set does not
appear in the output, which is what lets the value keep falling through
to a default class or to MuJoCo's own default, and it is why a second
write of the same document is a fixpoint.

## Related

- [The component model](model.md): how an element becomes a component.
- [Generation](generation.md): where the coverage comes from.
- [Importing Robots](../guides/importing.md): the import flow.
