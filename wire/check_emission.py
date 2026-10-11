#!/usr/bin/env python3
"""Compare what a handler EMITS against what the schema says the reply carries.

`gen_wire.py --check` proves the generated files match the schema. Nothing used
to prove the schema matches the code, so the schema could sit at a baseline the
handlers had long since moved past and every check stayed green: that is exactly
how `render_hello` came to advertise `ok`, `format`, `mjb`, `xml` and
`broadcasting` -- all deleted from the wire -- while omitting `features` and
`mujoco_version`, which the handler emits on every reply.

This walks the other link. For each op the schema declares a reply for, it finds
the C++ handler, reads back the top-level keys that handler sets on its reply
object, and diffs the two sets. It is deliberately a source reader rather than a
runtime probe: it must be able to fail on a machine with no engine, in the same
breath as `gen_wire.py --check`, which is where the drift would have been caught.

Static reading has limits and they are reported rather than hidden: an op whose
handler cannot be read unambiguously (a shared body that branches on the op name,
a reply assembled by a helper) is listed as UNREADABLE and checked at runtime
instead, by URLab.RpcFeature.ReplySchemaAllOps.

    ./wire/check_emission.py            # human-readable report, exit 1 on drift
    ./wire/check_emission.py --quiet    # only speak when something is wrong
"""
from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

try:
    import yaml
except ImportError:  # pragma: no cover
    sys.exit("check_emission.py needs pyyaml (pip install pyyaml)")

HERE = Path(__file__).resolve().parent
PLUGIN = HERE.parent
SCHEMA = HERE / "urlab_wire.yaml"

DISPATCHER = PLUGIN / "Source/URLab/Private/Bridge/RpcDispatcher.cpp"
HANDLER_DIR = PLUGIN / "Source/URLab/Private/Bridge"

# `Reg(TEXT("op"), ..., [](FURLabRpcDispatcher& D, auto& R) { return D.HandleX(R); }`
# is the only place an op name and its handler symbol are written together, so it
# is the map rather than the schema's `emit:` provenance, whose line numbers rot.
REG_RE = re.compile(
    r'Reg\(\s*TEXT\("(?P<op>[a-z0-9_]+)"\)\s*,'
    r'[^;]*?\{\s*return\s+(?:D\.)?(?P<fn>Handle\w+)\s*\(',
    re.DOTALL,
)
# A Reg() whose body is a named lambda declared elsewhere (RecBody / RepBody):
# one body serving several ops, branching on the op name inside.
REG_SHARED_RE = re.compile(
    r'Reg\(\s*TEXT\("(?P<op>[a-z0-9_]+)"\)\s*,[^;]*?,\s*(?P<body>\w+Body)\s*\)', re.DOTALL
)
FN_RE = re.compile(r'^TSharedPtr<FJsonObject> FURLabRpcDispatcher::(\w+)\(', re.MULTILINE)

# Only sets on the reply object itself count. A set on a nested object (VfsAssets,
# Instance, CamObj) is a field of that object, not a top-level wire key.
DIRECT_SET_RE = re.compile(r'\bReply\s*->\s*Set\w*Field\(\s*TEXT\("([^"]+)"\)')
UTIL_SET_RE = re.compile(r'FURLabMsgpackUtil::Set\w*Field\(\s*Reply\s*,\s*TEXT\("([^"]+)"\)')


def _function_bodies(path: Path) -> dict[str, str]:
    """Every FURLabRpcDispatcher method in a file, by name, brace-matched."""
    text = path.read_text(encoding="utf-8")
    out: dict[str, str] = {}
    for m in FN_RE.finditer(text):
        start = text.find("{", m.end())
        if start < 0:
            continue
        depth, i = 0, start
        while i < len(text):
            if text[i] == "{":
                depth += 1
            elif text[i] == "}":
                depth -= 1
                if depth == 0:
                    break
            i += 1
        out[m.group(1)] = text[start : i + 1]
    return out


def handler_bodies() -> dict[str, str]:
    bodies: dict[str, str] = {}
    for path in sorted(HANDLER_DIR.glob("Rpc*.cpp")):
        bodies.update(_function_bodies(path))
    return bodies


def op_to_handler() -> tuple[dict[str, str], set[str]]:
    """op -> handler symbol, plus the ops registered against a shared body."""
    text = DISPATCHER.read_text(encoding="utf-8")
    direct = {m.group("op"): m.group("fn") for m in REG_RE.finditer(text)}
    shared = {m.group("op") for m in REG_SHARED_RE.finditer(text)} - set(direct)
    return direct, shared


TAIL_CALL_RE = re.compile(r'return\s+(\w+)\s*\(')


def emitted_keys(body: str, bodies: dict[str, str], depth: int = 0) -> set[str]:
    """Top-level keys this handler sets on its reply, following one delegation.

    A handler that sets nothing and returns another dispatcher method (HandleHello
    -> BuildHandshakePayload) is a forwarder, not an empty reply, so the keys are
    read from where they are actually set.
    """
    keys = set(DIRECT_SET_RE.findall(body)) | set(UTIL_SET_RE.findall(body))
    if keys or depth > 2:
        return keys
    for callee in TAIL_CALL_RE.findall(body):
        if callee in bodies and callee != "MakeError":
            keys |= emitted_keys(bodies[callee], bodies, depth + 1)
    return keys


# --------------------------------------------------------------------------
# schema side
# --------------------------------------------------------------------------
def _resolve(msg: dict, where: str):
    """Follow a `reply_fields_at` path such as `reply.fields` into the message."""
    node = msg
    for part in where.split("."):
        if not isinstance(node, dict):
            return None
        node = node.get(part)
    return node


def schema_replies(schema: dict) -> dict[str, dict]:
    """op -> {'fields': [entry, ...], 'message': name} for every declared reply."""
    out: dict[str, dict] = {}
    for name, msg in (schema.get("messages") or {}).items():
        if not isinstance(msg, dict):
            continue
        # Strict typed lifecycle/domain replies are constructed as owned wire
        # values and validated against generated TypedResult metadata. This
        # source reader audits only the historical FJsonObject handlers.
        if msg.get("typed_dispatch"):
            continue
        op = msg.get("op")
        where = msg.get("reply_fields_at")
        if not op or not where:
            continue
        fields = _resolve(msg, where)
        if not isinstance(fields, list):
            continue
        out[op] = {"message": name, "fields": fields}
    return out


def reply_field_names(fields: list) -> list[str]:
    """Top-level wire keys only.

    `location:` marks a field that lives inside another object (hello's
    `capabilities` sits in `instance`), and a `[]` in the name marks an element
    of an array field. Neither is a top-level key, so neither is comparable to
    what the handler sets on the reply. `emitted_by:` names a field written by
    something other than the handler — a transport appending its own handshake
    block — which a source reader of the handler cannot see; it is an explicit,
    reviewable exemption rather than a silent gap.
    """
    names: list[str] = []
    for f in fields:
        if not isinstance(f, dict) or f.get("location") or f.get("emitted_by"):
            continue
        raw = f.get("name")
        for n in raw if isinstance(raw, list) else [raw]:
            if isinstance(n, str) and "[" not in n:
                names.append(n)
    return names


def main_check(quiet: bool = False) -> int:
    """Run the comparison. 0 when the schema and the handlers agree."""
    schema = yaml.safe_load(SCHEMA.read_text(encoding="utf-8"))
    declared = schema_replies(schema)
    handlers, shared_ops = op_to_handler()
    bodies = handler_bodies()

    checked: list[str] = []
    unreadable: list[tuple[str, str]] = []
    problems: list[str] = []

    for op in sorted(declared):
        want = set(reply_field_names(declared[op]["fields"]))
        fn = handlers.get(op)
        if op in shared_ops:
            unreadable.append((op, "registered against a shared body that branches on the op name"))
            continue
        if not fn:
            unreadable.append((op, "no Reg() site maps this op to a handler symbol"))
            continue
        body = bodies.get(fn)
        if body is None:
            unreadable.append((op, f"{fn} is not a FURLabRpcDispatcher method in Source/URLab/Private/Bridge"))
            continue
        got = emitted_keys(body, bodies)
        if not got:
            unreadable.append(
                (op, f"{fn} sets no key on a local `Reply`; its reply is assembled elsewhere"))
            continue
        checked.append(op)
        missing = sorted(got - want)   # emitted, undeclared
        extra = sorted(want - got)     # declared, never emitted
        if missing or extra:
            problems.append(
                f"  {op} ({fn}, message `{declared[op]['message']}`)\n"
                + (f"      emitted but NOT in the schema: {', '.join(missing)}\n" if missing else "")
                + (f"      in the schema but NOT emitted: {', '.join(extra)}\n" if extra else "")
            )

    if problems:
        print("SCHEMA/CODE DRIFT — the handler and wire/urlab_wire.yaml disagree.")
        print("The code is the wire. Bring the schema's field list to it, then re-run")
        print("wire/gen_wire.py and wire/sync_to_bridge.sh.\n")
        print("".join(problems))
        return 1

    if not quiet:
        print(f"emission check: {len(checked)} ops match the schema "
              f"({', '.join(checked)})")
        if unreadable:
            print(f"{len(unreadable)} declared ops are not statically readable; they are covered "
                  "at runtime by URLab.RpcFeature.ReplySchemaAllOps:")
            for op, why in unreadable:
                print(f"  {op}: {why}")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--quiet", action="store_true", help="print only on failure")
    args = ap.parse_args()
    return main_check(quiet=args.quiet)


if __name__ == "__main__":
    raise SystemExit(main())
