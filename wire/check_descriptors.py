#!/usr/bin/env python3
"""Cross-repo protobuf descriptor check.

The two repos each carry their own generated protobuf code for the same `.proto`
files: C++ under `Source/URLabDmEnvRpc/Private/GenProto/`, Python under
`URLab_Bridge/src/urlab_client/transports/_dmenv/`. They were produced by
different toolchains (see `toolchain.lock`), so the only thing worth asserting is
that the *descriptors* they embed are byte-identical — that is what guarantees
the two sides agree about the messages regardless of generator version.

Run it with both repos checked out side by side:

    python3 wire/check_descriptors.py

Exits non-zero and prints a diff summary on mismatch. Stdlib only apart from the
`protobuf` runtime, which the bridge already depends on.
"""
from __future__ import annotations

import hashlib
import importlib
import re
import sys
from pathlib import Path

# (C++ .pb.cc filename, C++ descriptor symbol infix, Python module name)
PAIRS = [
    ("urlab_dm_env_rpc.pb.cc", "urlab_5fdm_5fenv_5frpc_2eproto", "urlab_dm_env_rpc_pb2"),
    ("dm_env_rpc.pb.cc", "dm_5fenv_5frpc_2eproto", "dm_env_rpc_pb2"),
]

_SIMPLE_ESCAPES = {
    "n": 10, "t": 9, "r": 13, "\\": 92, '"': 34, "'": 39,
    "a": 7, "b": 8, "f": 12, "v": 11,
}


def _unescape_c_string(lit: str) -> bytes:
    """Decode one C string literal body to raw bytes (octal and hex included)."""
    out = bytearray()
    i = 0
    while i < len(lit):
        ch = lit[i]
        if ch != "\\":
            out.append(ord(ch))
            i += 1
            continue
        nxt = lit[i + 1]
        if nxt == "x":
            j = i + 2
            while j < len(lit) and lit[j] in "0123456789abcdefABCDEF":
                j += 1
            out.append(int(lit[i + 2:j], 16) & 0xFF)
            i = j
        elif nxt in "01234567":
            j = i + 1
            while j < len(lit) and j < i + 4 and lit[j] in "01234567":
                j += 1
            out.append(int(lit[i + 1:j], 8) & 0xFF)
            i = j
        elif nxt in _SIMPLE_ESCAPES:
            out.append(_SIMPLE_ESCAPES[nxt])
            i += 2
        else:
            out.append(ord(nxt))
            i += 2
    return bytes(out)


def cpp_descriptor(pb_cc: Path, symbol: str) -> bytes | None:
    """Extract the serialised FileDescriptorProto embedded in a generated .pb.cc."""
    if not pb_cc.is_file():
        return None
    text = pb_cc.read_text(encoding="utf-8", errors="surrogateescape")
    match = re.search(
        r"descriptor_table_protodef_" + symbol + r"[^\[]*\[\][^=]*=\s*\{(.*?)\};",
        text,
        re.S,
    )
    if not match:
        return None
    literals = re.findall(r'"((?:[^"\\]|\\.)*)"', match.group(1), re.S)
    return b"".join(_unescape_c_string(lit) for lit in literals)


def main() -> int:
    here = Path(__file__).resolve().parent          # <plugin>/wire
    plugin = here.parent
    bridge = plugin.parent / "URLab_Bridge"
    gen = plugin / "Source" / "URLabDmEnvRpc" / "Private" / "GenProto"

    if not bridge.is_dir():
        print(f"URLab_Bridge not found beside the plugin (looked in {bridge}).")
        print("This check needs both repos checked out side by side.")
        return 2

    sys.path.insert(0, str(bridge / "src"))
    venv = bridge / ".venv" / "lib"
    if venv.is_dir():                                # prefer the bridge's own venv
        for site in venv.glob("python3.*/site-packages"):
            sys.path.insert(0, str(site))

    failures = 0
    for cc_name, symbol, module_name in PAIRS:
        cpp = cpp_descriptor(gen / cc_name, symbol)
        try:
            module = importlib.import_module(
                f"urlab_client.transports._dmenv.{module_name}"
            )
            py = module.DESCRIPTOR.serialized_pb
        except Exception as exc:                     # noqa: BLE001 - reported, not raised
            print(f"FAIL {module_name}: python side did not import: {exc}")
            failures += 1
            continue

        if cpp is None:
            print(f"FAIL {module_name}: no descriptor found in {cc_name}")
            failures += 1
        elif cpp != py:
            print(f"FAIL {module_name}: descriptors differ")
            print(f"     C++    {len(cpp):>6} bytes  sha256 {hashlib.sha256(cpp).hexdigest()}")
            print(f"     Python {len(py):>6} bytes  sha256 {hashlib.sha256(py).hexdigest()}")
            print("     Regenerate both sides from the same .proto: wire/regen_proto.sh")
            failures += 1
        else:
            digest = hashlib.sha256(cpp).hexdigest()[:16]
            print(f"ok   {module_name:<24} {len(cpp):>5} bytes  sha256 {digest}...")

    if failures:
        print(f"\n{failures} descriptor mismatch(es).")
        return 1
    print("\nAll protobuf descriptors are byte-identical across the two repos.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
