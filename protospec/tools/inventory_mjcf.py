"""Inventory every workspace XML and classify MJCF with native MuJoCo.

This is a discovery ledger, not a UE pass. Every XML path is recorded,
including non-MJCF files, include fragments, invalid native models, and
duplicate copies. A separate UE corpus test consumes ``native_valid_paths``.

    uv run python tools/inventory_mjcf.py --workspace ../.. \
      --library /tmp/urlab-mujoco-pin-install/lib/libmujoco.so.3.11.1 \
      --plugins /tmp/urlab-mujoco-pin-build/lib --out /tmp/mjcf-inventory.json
"""

from __future__ import annotations

import argparse
import ctypes
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys
import xml.etree.ElementTree as ET


def xml_files(workspace: Path) -> list[Path]:
    result = subprocess.run(
        ["rg", "--files", "--hidden", "--no-ignore", "-g", "*.xml",
         "-g", "!**/.git/**", str(workspace)],
        check=True, capture_output=True, text=True)
    return sorted(Path(line).resolve() for line in result.stdout.splitlines())


def group(path: Path, workspace: Path) -> str:
    parts = path.relative_to(workspace).parts
    if parts[0] == "mujoco_menagerie":
        return "menagerie"
    if parts[:3] == ("UnrealRoboticsLab", "Content", "TestData"):
        return "urlab_testdata"
    if parts[:4] == ("UnrealRoboticsLab", "third_party", "MuJoCo", "src"):
        return "mujoco_source"
    if parts[:4] == ("UnrealRoboticsLab", "third_party", "install", "MuJoCo"):
        return "mujoco_install"
    return parts[0]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path, required=True)
    parser.add_argument("--library", type=Path, required=True,
                        help="source-matched native MuJoCo shared library")
    parser.add_argument("--plugins", type=Path,
                        help="directory of same-revision native plugin libraries")
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    workspace = args.workspace.resolve(strict=True)
    library = args.library.resolve(strict=True)
    native = ctypes.CDLL(str(library))
    native.mj_version.restype = ctypes.c_int
    native.mj_loadXML.argtypes = [ctypes.c_char_p, ctypes.c_void_p,
                                  ctypes.c_char_p, ctypes.c_int]
    native.mj_loadXML.restype = ctypes.c_void_p
    native.mj_deleteModel.argtypes = [ctypes.c_void_p]
    native.mj_loadPluginLibrary.argtypes = [ctypes.c_char_p]
    if args.plugins:
        for plugin in sorted(args.plugins.resolve(strict=True).glob("*.so")):
            if plugin.name != "libmujoco.so":
                native.mj_loadPluginLibrary(str(plugin).encode())

    entries = []
    duplicate_by_hash: dict[str, str] = {}
    for path in xml_files(workspace):
        rel = str(path.relative_to(workspace))
        contents = path.read_bytes()
        digest = hashlib.sha256(contents).hexdigest()
        entry = {"path": str(path), "relative": rel,
                 "group": group(path, workspace), "sha256": digest}
        if digest in duplicate_by_hash:
            entry["same_bytes_as"] = duplicate_by_hash[digest]
        else:
            duplicate_by_hash[digest] = rel
        try:
            try:
                root = ET.fromstring(contents)
            except LookupError:
                # Some non-MJCF XML in the workspace declares ISO-Latin-1,
                # which ElementTree's byte parser does not recognize. This
                # fallback only classifies the root; MuJoCo still parses the
                # original bytes for every MJCF candidate.
                text = contents.decode("latin-1")
                text = re.sub(r"^\s*<\?xml[^>]*\?>", "", text)
                root = ET.fromstring(text)
        except (ET.ParseError, UnicodeError, ValueError) as exc:
            entry.update(status="malformed_xml", error=str(exc))
            entries.append(entry)
            continue
        entry["root_tag"] = root.tag
        if root.tag == "mujocoinclude":
            entry["status"] = "include_fragment"
        elif root.tag != "mujoco":
            entry["status"] = "not_mjcf"
        else:
            entry["includes"] = [e.get("file", "") for e in root.iter("include")]
            entry["child_models"] = [e.get("file", "") for e in root.iter("model")]
            error = ctypes.create_string_buffer(8192)
            model = native.mj_loadXML(str(path).encode(), None, error, len(error))
            if model:
                entry["status"] = "native_valid"
                native.mj_deleteModel(model)
            else:
                entry.update(status="native_invalid",
                             error=error.value.decode("utf-8", "replace"))
        entries.append(entry)

    by_path = {entry["path"]: entry for entry in entries}
    for parent in entries:
        for kind, references in (("included_by", parent.get("includes", [])),
                                 ("model_referenced_by", parent.get("child_models", []))):
            for reference in references:
                target = str((Path(parent["path"]).parent / reference).resolve())
                if target in by_path:
                    by_path[target].setdefault(kind, []).append(parent["relative"])

    counts: dict[str, int] = {}
    groups: dict[str, dict[str, int]] = {}
    for entry in entries:
        status = entry["status"]
        counts[status] = counts.get(status, 0) + 1
        bucket = groups.setdefault(entry["group"], {})
        bucket[status] = bucket.get(status, 0) + 1
    output = {"workspace": str(workspace), "library": str(library),
              "library_sha256": hashlib.sha256(library.read_bytes()).hexdigest(),
              "mj_version": native.mj_version(), "counts": counts,
              "groups": groups, "entries": entries,
              "native_valid_paths": [e["path"] for e in entries
                                     if e["status"] == "native_valid"]}
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(output, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"total": len(entries), **counts}, sort_keys=True))
    print(args.out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
