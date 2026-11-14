#!/usr/bin/env python3
"""Compare the typed handlers against what wire/urlab_wire.yaml declares.

`gen_wire.py --check` proves the generated files match the schema. This proves
the schema matches the code, for every active typed op plus `connect` and
`disconnect`:

  * the op is handled: its name appears in its service's handler source (or it
    is the single remaining op in an exhaustive `if (Op == ...)` chain);
  * a code emitted at a site guarded by `Op == TEXT("<op>")` is declared on
    that op (or in `dispatcher_errors`, which any op may return);
  * every code emitted anywhere in a service's sources is declared on at least
    one op of that service, or in `dispatcher_errors`;
  * every code declared on an op is reachable from that op, so a declared list
    cannot keep codes the handler no longer produces;
  * `dispatcher_errors` lists exactly what the shared dispatch path emits.

It reads source, so it runs on a machine with no engine. A code is a string
literal passed as the code argument of `Error`, `ReplyError`, `MakeError` or
`ProtocolError`, assigned to a `Code` / `.Error.Code` field, returned from a
`*Error` / `*Code` mapping helper, or named through a `URLabError::` constant.
Reachability follows calls between functions of the service's sources. Result
shapes are not read here: every terminal result is validated against its
generated shape at runtime before it is retained.

    ./wire/check_emission.py            # report, exit 1 on drift
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
BRIDGE = PLUGIN / "Source/URLab/Private/Bridge"

SERVICE_SOURCES = {
    "render": [BRIDGE / "RpcHandlers_TypedRender.cpp"],
    "viewer": [BRIDGE / "RpcHandlers_TypedViewer.cpp",
               BRIDGE / "ViewerStreamConnection.cpp"],
    "streams": [BRIDGE / "RpcHandlers_TypedStreams.cpp",
                BRIDGE / "InteractionWire.h"],
    "simulation": [BRIDGE / "RpcHandlers_TypedSimulation.cpp"],
    "editor": [BRIDGE / "RpcHandlers_TypedEditor.cpp",
               PLUGIN / "Source/URLabEditor/Private/MjEditorOpHandlers.cpp"],
    "data": [BRIDGE / "RpcHandlers_TypedPayload.cpp"],
}
CONNECTION_OPS = ("connect", "disconnect")
DISPATCHER_SOURCES = [
    BRIDGE / "RpcHandlers_TypedConnection.cpp",
    BRIDGE / "RpcDispatcher.cpp",
    BRIDGE / "RetainedCompletion.cpp",
]
EDITOR_REGISTRY_PREFIX = "__typed_editor_"

CODE_RE = re.compile(r"^[a-z][a-z0-9_]*$")
TEXT_RE = re.compile(r'TEXT\("([^"\\]*)"\)')
ERROR_CALL_RE = re.compile(
    r"\b(?:Error|ReplyError|MakeError|ProtocolError|EncodeProtocolError)\s*\(")
CODE_ASSIGN_RE = re.compile(r"(?:\b(?:\w*ErrorCode|Code)|\.Error\.Code)\s*=(?!=)\s*")
OP_COND_RE = re.compile(r'\bOp\s*==\s*TEXT\("([a-z0-9_]+)"\)')
FUNC_HEAD_RE = re.compile(
    r"([A-Za-z_][\w:~]*)\s*\([^;{}]*\)\s*(?:const\s*)?(?:override\s*)?$", re.S)
IDENT_CALL_RE = re.compile(r"\b([A-Za-z_]\w*)\s*\(")
SCOPE_HEAD_RE = re.compile(r'(?:\b(?:namespace|class|struct)\b[\w\s:]*|extern\s+"C"\s*)$')
CONTROL_WORDS = {"if", "for", "while", "switch", "return", "sizeof", "TEXT", "check",
                 "ensure", "catch", "else", "do"}
NAMED_CODE_RE = re.compile(r"\bURLabError::(\w+)")
NAMED_CODES = dict(re.findall(
    r'inline const TCHAR\* (\w+) = TEXT\("([a-z0-9_]+)"\);',
    (PLUGIN / "Source/URLab/Public/Bridge/RpcErrorCodes.h").read_text(encoding="utf-8")))


def _blank_comments(text: str) -> str:
    """Blank comments and braces/semicolons inside literals, keeping offsets."""
    out = list(text)
    i, n = 0, len(text)
    while i < n:
        if text.startswith("//", i):
            j = text.find("\n", i)
            j = n if j < 0 else j
            out[i:j] = " " * (j - i)
            i = j
        elif text.startswith("/*", i):
            j = text.find("*/", i + 2)
            j = n if j < 0 else j + 2
            for k in range(i, j):
                if out[k] != "\n":
                    out[k] = " "
            i = j
        elif text[i] in "\"'":
            quote, j = text[i], i + 1
            while j < n and text[j] != quote:
                j += 2 if text[j] == "\\" else 1
            for k in range(i + 1, min(j, n)):
                if text[k] in "{};()" or quote == "'":
                    out[k] = " "
            i = j + 1
        else:
            i += 1
    return "".join(out)


def _split_top(args: str) -> list[str]:
    parts, depth, cur = [], 0, []
    for c in args:
        if c in "([{":
            depth += 1
        elif c in ")]}":
            depth -= 1
        if c == "," and depth == 0:
            parts.append("".join(cur))
            cur = []
        else:
            cur.append(c)
    parts.append("".join(cur))
    return parts


def _code_literals(expr: str) -> list[str]:
    """Code-shaped literals in an expression, skipping comparison operands."""
    out = []
    for m in TEXT_RE.finditer(expr):
        before = expr[:m.start()].rstrip()
        if before.endswith(("==", "!=", "Contains(", "StartsWith(", "EndsWith(")):
            continue
        if CODE_RE.match(m.group(1)):
            out.append(m.group(1))
    return out


class Source:
    """One C++ file: its brace blocks, functions, code sites and calls."""

    def __init__(self, path: Path, default_else_op: str | None = None) -> None:
        self.path = path
        self.default_else_op = default_else_op
        self.raw = path.read_text(encoding="utf-8")
        self.text = _blank_comments(self.raw)
        self.blocks: list[tuple[int, int, str]] = []
        stack: list[tuple[int, int]] = []
        stmt = 0
        self.stmt_start: list[int] = [0] * (len(self.text) + 1)
        for i, c in enumerate(self.text):
            self.stmt_start[i] = stmt
            if c == "{":
                stack.append((i, stmt))
                stmt = i + 1
            elif c == "}":
                if stack:
                    start, head_from = stack.pop()
                    self.blocks.append((start, i, self.text[head_from:start]))
                stmt = i + 1
            elif c == ";":
                stmt = i + 1
        self.blocks.sort()
        self.functions: list[tuple[str, int, int]] = []
        for start, end, head in self.blocks:
            enclosing = self._enclosing(start)
            if any(not SCOPE_HEAD_RE.search(h.strip()) for _, _, h in enclosing):
                continue
            m = FUNC_HEAD_RE.search(head.strip())
            if m and m.group(1).split("::")[-1] not in CONTROL_WORDS:
                self.functions.append((m.group(1), start, end))
        self.fallthrough_guards: list[tuple[int, int, frozenset[str]]] = []
        self._compute_fallthrough_guards()

    def _compute_fallthrough_guards(self) -> None:
        """Detect `if (Op == ...) ... else if (Op == ...) ... else { return; }` chains."""
        for _, fn_start, fn_end in self.functions:
            top = [b for b in self.blocks
                   if fn_start < b[0] < b[1] < fn_end
                   and self._enclosing(b[0])[-1][:2] == (fn_start, fn_end)]
            i = 0
            while i < len(top):
                s0, e0, h0 = top[i]
                ops0 = OP_COND_RE.findall(h0)
                if h0.strip().startswith("if") and ops0 and "!=" not in h0:
                    chain_ops: set[str] = set()
                    all_non_returning = True
                    j = i
                    while j < len(top):
                        sj, ej, hj = top[j]
                        hs = hj.strip()
                        if j > i and not hs.startswith("else"):
                            break
                        opsj = OP_COND_RE.findall(hj)
                        body = self.text[sj + 1:ej].strip()
                        ends_return = bool(re.search(r"\breturn\s*;\s*$", body))
                        if opsj and "!=" not in hj:
                            chain_ops.update(opsj)
                            if ends_return:
                                all_non_returning = False
                            j += 1
                        elif hs == "else" and ends_return and chain_ops and all_non_returning:
                            self.fallthrough_guards.append(
                                (ej, fn_end, frozenset(chain_ops)))
                            j += 1
                            break
                        else:
                            j += 1
                    i = max(i + 1, j)
                else:
                    i += 1

    def _enclosing(self, pos: int) -> list[tuple[int, int, str]]:
        return [b for b in self.blocks if b[0] < pos < b[1]]

    def function_at(self, pos: int) -> str:
        best = ("<file>", -1)
        for name, start, end in self.functions:
            if start < pos < end and start > best[1]:
                best = (name, start)
        return best[0]

    def ops_at(self, pos: int) -> frozenset[str] | None:
        """Ops named by the condition guarding `pos`, innermost first."""
        lead = self.text[self.stmt_start[pos]:pos]
        names = OP_COND_RE.findall(lead)
        if names and "!=" not in lead:
            return frozenset(names)
        enclosing = self._enclosing(pos)
        for _, _, head in reversed(enclosing):
            names = OP_COND_RE.findall(head)
            if names and "!=" not in head:
                return frozenset(names)
        if self.default_else_op:
            for start, end, head in enclosing:
                if head.strip().startswith("else") and not OP_COND_RE.findall(head):
                    return frozenset({self.default_else_op})
        for after_pos, fn_end, guard in self.fallthrough_guards:
            if after_pos < pos < fn_end:
                return guard
        return None

    def _args(self, open_paren: int) -> str:
        depth = 0
        for i in range(open_paren, len(self.text)):
            if self.text[i] == "(":
                depth += 1
            elif self.text[i] == ")":
                depth -= 1
                if depth == 0:
                    return self.raw[open_paren + 1:i]
        return self.raw[open_paren + 1:]

    def code_sites(self) -> list[tuple[int, str]]:
        sites: list[tuple[int, str]] = [
            (m.start(), NAMED_CODES[m.group(1)]) for m in NAMED_CODE_RE.finditer(self.text)
            if m.group(1) in NAMED_CODES]
        for m in ERROR_CALL_RE.finditer(self.text):
            args = _split_top(self._args(m.end() - 1))
            codes = _code_literals(args[0])
            if not codes and len(args) > 1 and args[0].strip().endswith("Request"):
                codes = _code_literals(args[1])
            sites += [(m.start(), c) for c in codes]
        for m in CODE_ASSIGN_RE.finditer(self.text):
            end = self.text.find(";", m.end())
            sites += [(m.start(), c) for c in _code_literals(self.raw[m.end():end])]
        for name, start, end in self.functions:
            if re.search(r"(Error|ErrorCode|AdmissionCode|TerminalCode)$", name.split("::")[-1]):
                for r in re.finditer(r"\breturn\b([^;]*);", self.text[start:end]):
                    expr = self.raw[start + r.start(1):start + r.end(1)]
                    codes = _code_literals(expr)
                    if codes == ["internal_error"] and re.search(
                            r"\bdefault\s*:\s*$", self.text[start:start + r.start()]):
                        continue
                    sites += [(start + r.start(), c) for c in codes]
        return sites

    def calls(self) -> list[tuple[int, str]]:
        out = [(m.start(), m.group(1)) for m in IDENT_CALL_RE.finditer(self.text)
               if m.group(1) not in CONTROL_WORDS]
        for m in re.finditer(r"\bFCompletionSink\b", self.text):
            out.append((m.start(), "CompleteInteraction"))
        for m in re.finditer(r"\bForwardInteraction\b", self.text):
            out.append((m.start(), "RunCommands"))
        return out


class Emission:
    """What a set of sources can emit, per op. Calls resolve across the set."""

    def __init__(self, sources: list[Source], ops: set[str]) -> None:
        self.guarded: dict[str, set[str]] = {op: set() for op in ops}
        self.reachable: dict[str, set[str]] = {op: set() for op in ops}
        self.unguarded: set[str] = set()
        self.all: set[str] = set()
        own: dict[str, set[str]] = {}
        names: dict[str, str] = {}
        for src in sources:
            names.update({name.split("::")[-1]: name for name, _, _ in src.functions})
        for src in sources:
            primary = src.path.name.startswith(("RpcHandlers_", "MjEditorOpHandlers",
                                                "RpcDispatcher", "RetainedCompletion"))
            for pos, code in src.code_sites():
                if primary:
                    self.all.add(code)
                guard = src.ops_at(pos)
                if guard is None:
                    own.setdefault(src.function_at(pos), set()).add(code)
                    continue
                for op in guard & ops:
                    self.guarded[op].add(code)
                    self.reachable[op].add(code)
        edges = []
        for src in sources:
            for pos, callee in src.calls():
                target = names.get(callee)
                caller = src.function_at(pos)
                if target and caller != "<file>" and target != caller:
                    edges.append((caller, target, src.ops_at(pos)))
        closure = {fn: set(codes) for fn, codes in own.items()}
        changed = True
        while changed:
            changed = False
            for caller, callee, guard in edges:
                if guard is not None:
                    continue
                codes = closure.get(callee, set())
                have = closure.setdefault(caller, set())
                if not codes <= have:
                    have |= codes
                    changed = True
        called = {callee for _, callee, _ in edges}
        for _, callee, guard in edges:
            if guard is not None:
                for op in guard & ops:
                    self.reachable[op] |= closure.get(callee, set())
        for fn, codes in closure.items():
            if fn not in called:
                self.unguarded |= codes
        for op in ops:
            self.reachable[op] |= self.unguarded


def _referenced(op: str, service: str, text: str, emission: Emission) -> bool:
    if f'TEXT("{op}")' in text or emission.guarded.get(op):
        return True
    return service == "editor" and op.startswith("editor_") and (
        f'"{EDITOR_REGISTRY_PREFIX}{op[len("editor_"):]}"' in text
        or f'TEXT("{op[len("editor_"):]}")' in text)


def main_check(quiet: bool = False) -> int:
    schema = yaml.safe_load(SCHEMA.read_text(encoding="utf-8"))
    dispatcher_declared = {e["code"] for e in schema.get("dispatcher_errors") or []}
    messages = [m for m in (schema.get("messages") or {}).values() if isinstance(m, dict)]
    typed = {m["op"]: m for m in messages if m.get("typed_dispatch") == "active"}
    connection = {m["op"]: m for m in messages if m.get("op") in CONNECTION_OPS}

    problems: list[str] = []

    def compare(op: str, declared: set[str], emission: Emission) -> None:
        undeclared = sorted(emission.guarded[op] - declared - dispatcher_declared)
        phantom = sorted(declared - emission.reachable[op])
        redundant = sorted((declared & dispatcher_declared) - emission.reachable[op])
        if undeclared:
            problems.append(f"  {op}: emitted but not declared: {', '.join(undeclared)}")
        if phantom:
            problems.append(f"  {op}: declared but never emitted: {', '.join(phantom)}")
        if redundant:
            problems.append(f"  {op}: already in dispatcher_errors: {', '.join(redundant)}")

    checked = 0
    dispatch = Emission([Source(p) for p in DISPATCHER_SOURCES], set(typed) | set(connection))
    emissions: dict[str, Emission] = {}
    texts: dict[str, str] = {}
    service_ops: dict[str, set[str]] = {}
    for service, paths in SERVICE_SOURCES.items():
        ops = {op for op, msg in typed.items() if msg["service"] == service}
        service_ops[service] = ops
        raw_all = "".join(p.read_text(encoding="utf-8") for p in paths if p.is_file())
        explicit = {op for op in ops if f'TEXT("{op}")' in raw_all}
        default_op = next(iter(ops - explicit)) if len(ops - explicit) == 1 else None
        sources = [Source(p, default_else_op=default_op if i == 0 else None)
                   for i, p in enumerate(paths) if p.is_file()]
        texts[service] = raw_all
        emission = Emission(sources, ops)
        for op in ops:
            emission.guarded[op] |= dispatch.guarded[op]
            emission.reachable[op] |= dispatch.guarded[op]
        emissions[service] = emission

    if "viewer" in emissions and "streams" in emissions:
        if "viewer_interaction_submit" in emissions["viewer"].reachable and (
                "interaction_submit" in emissions["streams"].reachable):
            emissions["viewer"].reachable["viewer_interaction_submit"] |= (
                emissions["streams"].reachable["interaction_submit"])

    for service in SERVICE_SOURCES:
        ops = service_ops[service]
        emission = emissions[service]
        text = texts[service]
        union: set[str] = set()
        for op in sorted(ops):
            checked += 1
            declared = {e["code"] for e in typed[op].get("errors") or []}
            union |= declared
            if not _referenced(op, service, text, emission):
                problems.append(f"  {op}: not referenced by any {service} handler source")
            compare(op, declared, emission)
        stray = sorted(emission.all - union - dispatcher_declared)
        if stray:
            problems.append(f"  {service}: emitted but declared on no op: {', '.join(stray)}")
    missing = sorted(set(typed) - {op for op, m in typed.items() if m["service"] in SERVICE_SOURCES})
    if missing:
        problems.append(f"  ops with no known service source: {', '.join(missing)}")

    for op in sorted(connection):
        checked += 1
        compare(op, {e["code"] for e in connection[op].get("errors") or []}, dispatch)
    stray = sorted(dispatch.unguarded - dispatcher_declared)
    if stray:
        problems.append(f"  dispatcher: emitted but not in dispatcher_errors: {', '.join(stray)}")
    unused = sorted(dispatcher_declared - dispatch.unguarded)
    if unused:
        problems.append(f"  dispatcher: in dispatcher_errors but never emitted: {', '.join(unused)}")
    if checked == 0:
        problems.append("  no ops were checked")

    if problems:
        print("SCHEMA/CODE DRIFT: the handlers and wire/urlab_wire.yaml disagree.")
        print("\n".join(problems))
        return 1
    if not quiet:
        print(f"emission check: {checked} ops match the schema")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--quiet", action="store_true", help="print only on failure")
    args = ap.parse_args()
    return main_check(quiet=args.quiet)


if __name__ == "__main__":
    raise SystemExit(main())
