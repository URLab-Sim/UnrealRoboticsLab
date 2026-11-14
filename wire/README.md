# `wire/` — the UE ↔ Python contract

Everything that crosses the boundary between this plugin and `URLab_Bridge` is
described here, in one place, on purpose.

## Why this exists

The two repos implement one protocol from two sides. Keeping a single machine-
checked schema prevents op names, request/result shapes, permission tables,
error vocabularies, capability tokens and command-line flags from drifting
between C++ and Python.

## What is here

| File | Role |
|---|---|
| `urlab_wire.yaml` | The schema. Every message, request/result shape, permission, error list, unit, enum and binary layout. |
| `gen_wire.py` | Emits the shared vocabulary, request/result shapes, permission table (`OP_PERMISSIONS` / `URLabWire::OpPermission`), per-op and dispatcher error tables (`TYPED_OP_ERRORS`, `DISPATCHER_ERRORS`), capability words and flag metadata into both repos. `--check` fails when either side has drifted or when `check_emission.py` fails. |
| `check_emission.py` | Reads the C++ dispatcher and service handlers (`RpcHandlers_Typed*.cpp`, `MjEditorOpHandlers.cpp`, `RpcDispatcher.cpp`, `RetainedCompletion.cpp`) and fails when the ops or error codes they emit disagree with `urlab_wire.yaml`. |
| `toolchain.lock` | Which protobuf/gRPC toolchains produced the checked-in generated code, and the known skew between them. |
| `check_descriptors.py` | Proves the two repos' protobuf descriptors are byte-identical. |
| `regen_proto.sh` | Regenerates both sides from the one set of `.proto` files, using the pinned toolchains. |
| `sync_to_bridge.sh` | Vendors the schema and lock into the bridge and records their digests. |

The bridge carries a read-only copy in its own `wire/` plus a `SCHEMA.lock`, and
`tests/test_wire_schema.py` there fails if the copy is edited or goes stale.

## Working on the wire

**Changing a message or shape.** Edit `urlab_wire.yaml`, then run
`./wire/sync_to_bridge.sh`. Do not edit the bridge's copy; it is generated and
its test will tell you so.

**Changing a `.proto`.** Run `./wire/regen_proto.sh`, which regenerates both sides
with the pinned toolchains and then verifies the descriptors still match. Never
regenerate one side alone.

**Checking without changing anything.** `./wire/gen_wire.py --check` verifies the
generated C++ and Python wire files and runs `check_emission.py`.
`./wire/regen_proto.sh --check` verifies the protobuf descriptors.

## The links that are checked

**Schema → generated files.** `gen_wire.py --check` fails when the emitted
`generated.py` or `UrlabWire.gen.h` no longer match `urlab_wire.yaml`.

**Schema → handler code (static).** `check_emission.py` checks all 76 ops
(`74` active typed ops plus `connect` and `disconnect`): every op must be
handled by its service's C++ source, every error code emitted for an op must be
declared in that op's `errors:` list or in `dispatcher_errors`, every code
declared on an op must be reachable from that op's handler, and
`dispatcher_errors` must match what the shared dispatch path emits.

**Schema → runtime payloads.** `FURLabOperationSchema::ValidateRequestBody` and
`ValidateResultBody` (called in `FURLabRetainedCompletion::Publish`) validate
every typed request and terminal result against the generated shapes before
retention, and Python `Connection` validates terminal results against
`TYPED_RESULT_SHAPES`.

## Permissions and errors

Each op in `messages` declares its required `permission:` (`none`, `camera_rpc`,
`accept_input`, `accept_model_load`, `accept_editor_commands`). `gen_wire.py`
derives `OP_PERMISSIONS` and `URLabWire::OpPermission` directly from those
per-op declarations.

Shared dispatch failures (envelope validation, session expiry or boot mismatch,
adapter filtering, data negotiation, request ledger capacity/conflict/retirement,
result validation/encoding) live in `dispatcher_errors`. Operation-specific
failures live in each op's `errors:` list.
