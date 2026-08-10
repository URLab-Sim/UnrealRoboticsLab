# Architecture: from components to a simulation

This page explains how a robot you edit in the Unreal editor becomes a model
MuJoCo simulates, and why the parts that look strange are the way they are. It
is for someone about to change that code. Nothing here is optional trivia: every
section exists because getting it wrong produces a model that compiles, runs,
and is quietly wrong.

## The shape in one paragraph

An Unreal Blueprint holds a tree of components. That tree **is** the authored
document — there is no MJCF text behind it. To simulate, the tree is walked once
into an `mjSpec`, MuJoCo's own in-memory model description, and MuJoCo compiles
that into an `mjModel`. Identity is carried forward by handle and then by
integer id, never by name. MJCF text appears in only two places, both of them
edges of the system.

```
component tree  ──BuildSpec──▶  mjSpec  ──mj_compile──▶  mjModel  ──▶  simulation
      │                                                      │
      │ MJCF text at the file boundary only                  │ ids, never names
      └── import / export / handshake writer ────────────────┘
```

## The two seams

MJCF text exists at exactly two boundaries. Everywhere else, text is a bug.

**1. The file boundary.** `MjMjcfIo_Scs.cpp` and `MjMjcfIo_Instance.cpp` read
MJCF into components and write components back out. The handshake writer
(`MjSceneMjcf.cpp`, reached from `MjPhysicsEngine.cpp`) serializes the assembled
scene to MJCF for external clients. This is where text belongs.

**2. The macro bridge.** A few MJCF constructs are not elements at all but
compile-time macros — `<replicate>`, `<composite>`, `<flexcomp>` — which MuJoCo
expands during its own read, not through the `mjSpec` API. There is no
`mjs_addReplicate`. So for those, and only those, the build serializes the
subtree back to MJCF text and hands it to MuJoCo's reader. That is what
`MjSpecWriteHooks.cpp` calls the wrapper path, and it is scoped to the macro's
subtree.

Two things about the wrapper document are load-bearing and were established by
running the engine, not by reading it:

- **The carrier is a `<frame>`, never a `<body>`.** The macro sits under a frame
  carrying the class its enclosing body imposed. A frame is flattened at
  compile and a body is not, so a body carrier would put an extra link in the
  kinematic chain and shift every name after it.
- **The wrapper carries the asset section minus the names the target already
  holds.** The expansion is attached with an empty prefix, and `mjs_attach`
  rejects a repeated asset name outright — so carrying the section wholesale
  fails the attach, which is unrecoverable (see below). It is a set difference
  rather than a walk over what the macro references, and that is what makes it
  safe: anything omitted is omitted *because* the target has it, so no omission
  can break a reference. The wrapper spec is parsed and never compiled, so the
  references it leaves dangling resolve in the target after the attach.

If you find yourself producing MJCF text anywhere else, the design has been
misunderstood.

## The generated profile, and how narrow its seam is

The MJCF grammar is upstream's (`src/xml/mjcf.schema` in the MuJoCo submodule).
ProtoSpec — the plugin's `protospec/` directory — reads it with MuJoCo's own
parser, applies an overlay of corrections and classifications, and emits a
typed C++ object model into `Source/URLab/*/MuJoCo/Gen`. That is 145 elements
and 1,533 attributes of generated code.

Hand-written code never names a generated element type. It goes through ten
entry points, aliased in one file (`MjGenHooks.h`): the per-element field visit,
type/id conversions in both directions, node dispatch, the child-slot queries,
the string and shape storage policies, and the schema defaults. A rename on the
generated side is one edit because the seam is one file wide.

`URLAB_MJ_GEN` is 1 when ProtoSpec and the generated tree are both present.
Everything depending on generated types is gated on it — which means a checkout
missing them **builds and passes tests** with the whole pipeline compiled out.
See the [bump guide](contributing/bumping_mujoco.md) for how to prove it is
actually there.

### The overlay is the only hand-maintained table

Corrections and classifications live in `protospec/protospec_gen/overlay.py` and
`overlay_ue.py`. Every table is gated in both directions: an entry naming
something the schema no longer declares fails generation, and an addition the
overlay has not classified fails generation too. The three addition gates exist
because three kinds of upstream addition would otherwise land silently wrong —
an interleaved child in the wrong order shifts ids, an unclassified angle
attribute is wrong by 57.3x, an unclassified dynamic reference is invisible to
the rename and referrer machinery. The bump guide covers answering them.

## The build walk

`BuildSpec` (`MjSpecBuild.cpp`) walks the component tree once and writes an
`mjSpec`. Three properties of that walk matter more than they look.

**Sections are written in a fixed phase order, not document order.**
`PhaseOf` fixes it: defaults, then the model-level blocks, then assets, then
bodies, then contacts, deformables, equalities, tendons, actuators, sensors,
custom, keyframes, extensions. It cannot be document order, because a `<geom>`
naming a material has to find that material already on the spec, and MuJoCo
resolves both material references and default classes by name at compile time.
The order rests on the MJCF assumption that definitions precede use — which is
also how MJCF is conventionally written by hand, so it is not arbitrary, but it
is an assumption and nothing checks it.

**Inside a `<default>` class, nested classes are written last.** Schema order
puts `<default>` first among a class's children. `mjs_addDefault` copies the
enclosing class's templates into the new class *at the moment it is created*, so
opening a nested class before its parent's templates have been written means the
nested class inherits nothing. The walk therefore stable-sorts nested defaults
to the end. The defaults area has produced more silent-wrongness bugs than any
other part of this code; treat a change there as high risk.

**Element identity is recorded during the walk, not recovered after.**
`FMjBuiltSpec` carries `ElementFor`, a map from component to `mjsElement*`,
valid exactly as long as the spec is. Components store an integer id and never a
pointer, so no pointer into a spec can outlive it through a component.

### The hook contract

Attributes MuJoCo's reader *interprets* rather than stores — `fromto` folding to
pos/quat/size, transmission election on an actuator, the equality operand
election, hfield row handling — have no plain `mjs_set*` path. For each,
`MjSpecWriteHooks.cpp` reproduces the rule MuJoCo's own reader applies, and
cites the upstream source location it mirrors. About 2,000 lines work this way.

The contract for anyone touching them:

- The citation is part of the code. If you change a hook, re-read the upstream
  location and update the citation.
- Nothing automatically checks a citation still describes the code it names, so
  re-reading them is a manual step of every MuJoCo bump.
- A hook that gets a rule wrong produces a model that compiles. The live parity
  check (below) is what catches it.

## Names on the wire: the `_ps:` prefix

MJCF lets elements be unnamed. Several things downstream cannot work with
unnamed elements — most importantly the Python bridge, which reconciles the
model it holds against the one Unreal is simulating.

So during a build, unnamed components are temporarily given generated names
beginning `_ps:` (`MjReservedNames.h`). Three things about this are
load-bearing:

**It is a wire contract, not an implementation detail.** These names leave the
engine: they are in the compiled model, in `scene_compiled.xml`, and in the
handshake MJCF. Remote clients reconcile by them. Changing the prefix or the
ordinal allocation breaks those clients — this is a cross-repository protocol
change, not a plugin change.

**The name is a fact about the document, not about the session.** A reserved
name reads `_ps:<family>:<n>`, where `n` counts the reservations of that family
in document order within the spec being named, from one. The same spec therefore
reserves the same names in every process: a recorded `.mjb` verifies against a
later run, `scene_compiled.xml` diffs quietly against the one before it, and a
bridge client's names survive a restart. The ordinal deliberately does not come
from `UMjNodeComponent::Serial`, which is a process counter and cannot repeat.
Uniqueness *across* specs is participant prefixing's job, not this one's.

**The names go on the components, transiently.** They are written onto the
components' `MjName` and removed at scope exit, and the tree is handed back
exactly as it was found. They have to be on the components rather than on the
built elements because the macro bridge re-serializes subtrees *from the
components*, so an unnamed body inside a `<replicate>` is named in the expansion
only because its component temporarily carries the name. The consequence to know:
during a build, the component tree briefly holds names the user never authored.
Anything observing components mid-build — another thread, a callback, an editor
tick — sees state the owner never authored.

**Retirement condition.** This mechanism can be removed when nothing outside the
engine looks elements up by generated name; that means the bridge reconciling by
stable identity (ids carried in the handshake) instead of by name. Until then it
stays, and it is pinned by an explicit test so it cannot drift.

## Assets: three conventions, deliberately distinct

Three different destinations for asset files exist and they are **not**
interchangeable. Confusing them is a recurring bug, so each is named here with
what it is for.

| Convention | Where | For |
|---|---|---|
| `urlab_assets/` | on disk, inside the directory MJCF already resolves the element's `file` against | **Export.** Meshes written back as OBJ, textures as PNG, so the written spec stays portable and relative. |
| prefixed basenames | in the `mjVFS`, in memory | **Compile.** Assets mounted for `mj_compile`, one flat namespace. |
| `scene_assets/` | on disk, beside a debug dump | **Diagnostics.** `SaveDebugArtifacts` only. |

A change to one sink must be scoped to that sink.

### Why the VFS uses prefixed basenames, not directories

MuJoCo's VFS falls back to a **case-insensitive basename match across every
mount**. Two participants in a scene each carrying their own `base.obj` would
otherwise silently resolve to whichever was mounted first. Prefixing the
basename gives each participant its own namespace inside one flat VFS.

### Derive the name, then prefix — in that order

An asset that authored no name gets the name MuJoCo *would* derive for it, and
gets it explicitly, **before** the file reference is rewritten. MuJoCo derives an
unnamed asset's name from its file and then prefixes the result. Rewrite the file
first and the derivation runs over an already-prefixed basename: `p0_base.obj`
becomes mesh `p0_p0_base`, while the geom referring to it was prefixed once to
`p0_base`, and the compile fails on a reference to nothing.

### Asset syncing is not part of `Compile()`

`MjSyncAssetFiles` runs at install time in `MjPhysicsEngine`, not inside
`Compile()`. Code that composes scenes through the scene-spec builder directly —
as tests do — silently skips asset syncing.

## Scene composition and attach

A scene is assembled by attaching participant specs into a scene root with
`mjs_attach`. One rule dominates:

!!! danger "A failed `mjs_attach` corrupts the target spec beyond recovery"

    There is no partial attach and no retry. On failure the build is abandoned
    and the whole spec discarded. Do not weaken that path, and do not add a
    "try the next participant" branch — the spec it would write into is already
    unusable.

    The error path reads diagnostics out of the spec the attach just corrupted.
    That is survivable only because the build is abandoned immediately
    afterwards; anything that later touches that spec inherits an unusable one.

Deep copy is raised only around the nested-model attach. Participant attach and
the macro bridge attach deliberately without copying.

### Conflicts between a participant and the scene are MuJoCo's to resolve

Two documents composed into one both carry model-level blocks, and only one set
survives. That decision is not ours: MuJoCo has a conflict resolver
(`user_api.cc`) that runs inside every `mjs_attach`, driven by `mjtConflict` on
the compiler block — `warning`, `merge` or `error` — which the scene author sets
in MJCF like any other attribute.

What it visits is `<option>`, `<visual>` and `<size>`. It never visits
`<compiler>`, so a policy is read from there but nothing in there is resolved.

**The resolver only sees values it knows were authored**, and "authored" is a
per-field flag MuJoCo's own reader sets. A value a document wrote out
explicitly that happens to equal MuJoCo's default is invisible to the resolver
without it, so the spec build sets the flags itself, from the authored state the
components already know. The two calls that do it, `mjs_setAuthored` and
`mjs_isAuthored`, are exported from the library and absent from its public
headers, so both are declared by hand (`MjAuthored.h`, mirroring
`src/user/user_api.h`) and a test exercises them end to end — a signature change
upstream would otherwise be found by a user, not by a compiler.

Flags go on **every** spec the build produces, in one change and never half.
A flagged spec attached into an unflagged one can swallow a real conflict even
under the error policy, which is worse than flagging nothing.

!!! note "Mixed units across an attach are safe by MuJoCo's design"

    A robot authored in degrees dropped into a radians scene compiles with
    correct geometry, and no policy setting is needed to make it so. Attached
    elements keep a pointer to the document that authored them and compile under
    *its* angle setting. This was proven with both pre-resolved and unresolved
    orientations. There is no 57x hazard here to defend against, and the policy
    mechanism could not express one anyway.

## The correctness nets

Four independent nets cover different failure shapes. Know which one would have
caught your bug.

**Live compile parity** compiles every parity fixture two ways — through the
component tree and `BuildSpec`, and through `mj_loadXML` of the same authored
file — and field-diffs the two `mjModel`s at zero tolerance. Nothing is recorded,
so it cannot bless a regression, and it is the net that keeps the hand-written
hooks honest. This is the one to trust.

**Compiled-model goldens** (`Content/TestData/goldens/`, `.mjb` files) pin
output across changes that are not supposed to move it. They are committed and
protected: a changed golden stops the change until someone explains it.

`mj_loadModel` validates the MuJoCo version in the `.mjb` header, so every
golden fails to load after any MuJoCo release bump — with a message
indistinguishable from file corruption. `CAPTURE.json` beside the goldens
records the version and submodule SHA they were captured at, and the test reads
it first so the failure explains itself. Capture mode writes only files that are
**absent**, so recapture is delete-then-run, and the live parity check above is
what validates the new recording.

**The corpus net** (`protospec/corpus_net.ps1` / `.sh`) round-trips every model
in MuJoCo's own test corpus through the reader and writer and diffs the result
against a stock load. It has a small recorded allowed-failure list; adding to it
is a decision. `build_and_test` does not run it.

**`URLab.Gen.Profile.HandshakeMjcfEquivalence`** is the only thing tying the
handshake MJCF to the compiled model. They are two serializations of one scene,
produced by different code, and the compile never uses the text. Anyone changing
the writer or the spec walk needs to know that test is the sole tie.

## Landmines

Verified, each with a failure that is quiet rather than loud.

**`mjs_setDefault` on a `<default>` element writes over the wrong memory.**
Every element but one resolves *through* a class and carries the name of the
class it resolved through. A `<default>` **is** a class: its handle is the class
object, which has no class-name field, so `mjs_setDefault` would put a name over
whatever member sits at that offset. The build excludes `Default` from that call
explicitly. Do not "simplify" the exclusion away.

**`mjCModel::CopyList` silently drops elements with unresolved references.**
Not an error, not a warning: fewer elements. Anything relying on a copied list
being complete has to establish that separately.

**`WrapperNeedsAsset` omits unnamed assets from macro wrappers, by design.**
An unnamed element's compiled name is derived by MuJoCo from its file, so a
second copy inside the wrapper collides at compile rather than at attach.
Correctness depends on the sink-derived name already being pinned onto the
target-spec element (see derive-then-prefix, above), or on the compile-time
derivation matching. That reasoning lives in comments at the site and nowhere
else, so read it before changing the wrapper.

**`Utf8()` is an eight-slot thread-local ring.** It is a ring rather than a
single buffer because several conversions appear as separate arguments of one
`mjs_*` call and each must survive until that call runs. A hook that keeps more
than eight conversions alive across one call dangles silently. Eight is not a
round number chosen for comfort; it is the headroom over the widest call in the
tree today.

**A rigid flex on a static body, touching another static body, aborts inside
`mj_forward` — upstream, and not at load.** The contact row's Jacobian is all
zeros, `treeIterInit` takes the generic-scan branch for flex contacts
(`engine_island.c`), the scan finds no tree, and `unionConstraintTrees` raises
`mj_island: no tree found for constraint 0` through MuJoCo's fatal-error
handler. MuJoCo's own test suite excludes this shape from its parity tests. The
consequence for our tooling: anything that steps an arbitrary model has to trap
the fatal handler, or the process dies and the failure gets attributed to
whatever ran last. The round-trip harness does trap it and compares every model
field regardless, skipping only the step invariants for a model whose forward
pass aborts — the original file fails identically under stock MuJoCo, which is
how we know the reader and writer are not involved.

**`mjtSize` is not `int32`.** Comparing one against an `int32` in a `TestEqual`
is an ambiguous-overload compile error (C2666), not a warning. Cast at the call
site. Three builds have died on this.

**`SiblingIndex` is stamped only by the tree adapter's `Adopt`.** Spec order is
qpos order. Any path that creates spec components without going through the tree
adapter produces wrong spec order — silently, because the model still compiles.

**Name reconciliation happens once, at import.** It is safe only on a fresh
asset, by its own contract. Everything downstream assumes `MjName` and the
strings that refer to it stay consistent, which is user discipline plus the
edit-time diagnostics, not an invariant.

**An empty automation test log has three meanings**, and they are
indistinguishable from each other and from a clean pass if you only check the
exit code: the build failed before tests ran, the editor could not take the
project lock, or the run crashed before writing. A run counts only when the log
carries `Automation Test Queue Empty N tests performed` at the expected `N`.
Zero failures without that line is a crash.

## Where things are

| Path | What |
|---|---|
| `Source/URLab/Private/MuJoCo/Spec/` | the build walk, hooks, scene composition, asset sink, MJCF IO |
| `Source/URLab/Public/MuJoCo/Spec/MjGenHooks.h` | the ten-entry-point seam to the generated profile |
| `Source/URLab/*/MuJoCo/Gen/` | generated, checked in, never hand-edited |
| `protospec/protospec_gen/` | the generator and its overlay |
| `third_party/MuJoCo/src/src/xml/mjcf.schema` | upstream's grammar, the source of truth |
| `Content/TestData/parity/`, `goldens/` | the fixtures and recordings the nets run on |

## Related

- [Architecture](concepts/architecture.md): the other half — how the compiled
  model is stepped, the physics thread, the render snapshot, the transports.
  This page stops where that one starts.
- [Bumping MuJoCo](contributing/bumping_mujoco.md): the procedure that keeps all
  of the above in step with a new engine version.
- [Codegen](concepts/codegen.md): the generator in more detail.
