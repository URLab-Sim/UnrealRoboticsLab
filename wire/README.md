# `wire/` — the UE ↔ Python contract

Everything that crosses the boundary between this plugin and `URLab_Bridge` is
described here, in one place, on purpose.

## Why this exists

The two repos implement one protocol from two sides. Until this directory
existed, each side kept its own hand-typed copy of the op names, the message
fields, the flag spellings and the capability words — **seven independent copies
in total**, and nothing compared them. The results were exactly what you would
predict:

- the Python farm launcher spent four days emitting three command-line flags the
  plugin had deleted, so every farm node booted on the same port, while the unit
  test asserted the dead spelling was correct;
- the two protobuf toolchains drifted four major versions apart unnoticed;
- one field, `vfs_assets`, ended up with two different encodings because Python
  was written to match an *internal* Unreal representation that had leaked onto
  the wire.

A rename can only propagate by hand, and a hand that misses is never
contradicted. This directory is the fix for that class of bug.

## What is here

| File | Role |
|---|---|
| `urlab_wire.yaml` | The schema. Every message, field, unit, enum and binary layout. |
| `gen_wire.py` | Emits the shared vocabulary, the render-bus topic grammar and the op registry's field declarations into both repos. `--check` fails when either side has drifted. |
| `check_emission.py` | Reads the C++ handlers back and fails when what they emit no longer matches the schema. |
| `toolchain.lock` | Which protobuf/gRPC toolchains produced the checked-in generated code, and the known skew between them. |
| `check_descriptors.py` | Proves the two repos' protobuf descriptors are byte-identical. |
| `regen_proto.sh` | Regenerates both sides from the one set of `.proto` files, using the pinned toolchains. |
| `sync_to_bridge.sh` | Vendors the schema and lock into the bridge and records their digests. |

The bridge carries a read-only copy in its own `wire/` plus a `SCHEMA.lock`, and
`tests/test_wire_schema.py` there fails if the copy is edited or goes stale.

## Reading the schema

`urlab_wire.yaml` began as a transcription of the wire exactly as it behaved,
disagreements included, because an honest baseline is what later steps diff
against. Much of it is now a design: `capability_model` in particular describes
what both sides implement, and both sides generate from it.

So `drift:` entries are expected and are the point. Each one carries prose
describing the disagreement, the `file:line` on both sides, and a
`resolution:` that reads `undecided` until the divergence-resolution step
settles it. Field `status:` is one of:

- `ok` — both sides agree
- `drift` — they disagree, see the adjacent `drift:` block
- `dead` — someone emits it, nobody reads it
- `phantom` — someone reads it, nobody emits it

There are real examples of all four. `t`, the frame timestamp, is a `phantom`:
the subscriber reads it, no owner has ever sent it, and that is why sim-clock camera
delay silently does nothing on the streaming path.

## Working on the wire

**Changing a message.** Edit `urlab_wire.yaml`, then run `./wire/sync_to_bridge.sh`.
Do not edit the bridge's copy; it is generated and its test will tell you so.

**Changing a `.proto`.** Run `./wire/regen_proto.sh`, which regenerates both sides
with the pinned toolchains and then verifies the descriptors still match. Never
regenerate one side alone — that is how the version skew happened.

**Checking without changing anything.** `./wire/regen_proto.sh --check` verifies
the descriptors and generates nothing. It works on a machine with neither
toolchain installed.

## The two links, and why both are checked

There are two ways this can rot, and until recently only one of them was
watched.

**Schema → generated files.** `gen_wire.py --check` fails when the emitted
`generated.py` or `UrlabWire.gen.h` no longer match the schema. This was always
here.

**Schema → code.** Nothing compared the schema against the handlers, so a
handler could be rewritten and the check stayed green on a description of a wire
that no longer existed. That is exactly what happened to `render_hello`: it
was rewritten to emit `features` and `mujoco_version` and to drop `ok`,
`format`, `mjb`, `xml` and `broadcasting`, and the schema — and therefore the op
registry's declaration, and therefore the shape the `meta` op handed to clients —
went on describing the old one. `check_emission.py` walks that link, and
`gen_wire.py --check` now runs it. A third pass runs at runtime, inside Unreal:
`URLab.RpcFeature.ReplySchemaAllOps` dispatches every op it can reach and holds
each reply against its declaration, which catches the handlers a source reader
cannot follow.

## Declaring an op

A message that carries `reply_fields_at:` (naming the key that holds its reply
field list) generates the op registry's `ReplyFields`, and its Reg() site in
`RpcDispatcher.cpp` carries no literals at all. `request_fields_at:` does the
same for `RequiredFields`, from fields marked `registry_required: true`.

## Declaring an op's authority

`capability_model.op_permissions` says which permission each op requires. The
dispatcher enforces it in one place, before the handler runs, for every op --
including the pre-session fast-path family, which returns before the registry is
consulted at all. An op absent from the table requires none, and that is the
DECLARED default rather than an omission: `URLab.Wire.OpPermissionCoverage`
counts the gated ops and fails on a table entry that names no registered op.

A check written inside a handler is the thing this replaces. It is invisible to a
caller, the next handler forgets it, and in the model swap's case it was simply
never written -- which is how a discovery probe could replace a running scene.

`registry_required` is deliberately NOT the same as `required`. `required` is a
protocol statement; `registry_required` says the dispatcher rejects the request
declaratively, before the handler runs. Several protocol-required fields are
validated inside their handler on purpose, so the caller gets a specific
`bad_request` instead of the generic missing-field envelope, and conflating the
two would change behaviour.

An op with no `reply_fields_at:` keeps a hand-written list at its Reg() site.
That is allowed, and counted: `URLab.Wire.OpDeclarationCoverage` reports how many
remain so the number can only fall deliberately.

Two field markers exempt an entry from the static emission check, both of them
facts rather than escape hatches: `location:` says the field lives inside another
object (hello's `capabilities` sits in `instance`), and `emitted_by:` names a
field written by something other than the handler (a transport appending its own
handshake block).
