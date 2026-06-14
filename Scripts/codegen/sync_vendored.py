# Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
"""
Refresh vendored third-party files listed in
``Scripts/codegen/_vendored/_VENDORED_FROM.md``.

Manifest format: GitHub-flavoured markdown table with columns
``file | upstream_url | upstream_sha | retrieved_date | license | local_dest``.
``upstream_url`` may contain ``{sha}`` which gets substituted with
``upstream_sha`` at fetch time.

Re-running on the same SHAs produces zero diff in the manifest (the
script rewrites the table with stable column ordering + sorted-by-dest
rows, and re-stamps ``retrieved_date`` only when the fetch changed the
file content).

Usage:
    python Scripts/codegen/sync_vendored.py
        [--manifest Scripts/codegen/_vendored/_VENDORED_FROM.md]
        [--dry-run]   (don't write anything; print what would happen)
"""

from __future__ import annotations

import argparse
import datetime as _dt
import hashlib
import os
import re
import sys
import urllib.request
from typing import Dict, List

_HERE = os.path.dirname(os.path.abspath(__file__))
_PLUGIN_ROOT = os.path.normpath(os.path.join(_HERE, "..", ".."))
_DEFAULT_MANIFEST = os.path.join(_HERE, "_vendored", "_VENDORED_FROM.md")


# Markdown-table row: pipe-separated, with leading + trailing pipes
# trimmed. Comments / preamble paragraphs are ignored.
_ROW_RE = re.compile(r"^\s*\|(?P<row>.+)\|\s*$")


def _parse_manifest(text: str) -> List[Dict[str, str]]:
    """Return the table rows as dicts keyed by column header. Skips the
    header row and the markdown separator row (``|---|---|...``)."""
    rows: List[Dict[str, str]] = []
    header: List[str] = []
    for line in text.splitlines():
        m = _ROW_RE.match(line)
        if not m:
            continue
        cols = [c.strip() for c in m.group("row").split("|")]
        if not header:
            header = cols
            continue
        if all(c.startswith("-") or c == "" for c in cols):
            continue  # separator
        if len(cols) != len(header):
            continue  # malformed; ignore quietly
        rows.append(dict(zip(header, cols)))
    return rows


_COL_ORDER = ("file", "upstream_url", "upstream_sha", "retrieved_date",
              "license", "local_dest")


def _format_manifest(rows: List[Dict[str, str]], preamble: str) -> str:
    """Render the table with stable column ordering + sorted-by-dest rows."""
    rows = sorted(rows, key=lambda r: r.get("local_dest", ""))
    header = " | ".join(_COL_ORDER)
    sep = " | ".join("-" * max(3, len(c)) for c in _COL_ORDER)
    lines = [f"| {header} |", f"| {sep} |"]
    for r in rows:
        cells = [r.get(c, "") for c in _COL_ORDER]
        lines.append("| " + " | ".join(cells) + " |")
    return preamble.rstrip() + "\n\n" + "\n".join(lines) + "\n"


def _extract_preamble(text: str) -> str:
    """Strip everything from the first table row onward."""
    lines = text.splitlines()
    out: List[str] = []
    for line in lines:
        if _ROW_RE.match(line):
            break
        out.append(line)
    return "\n".join(out)


def _fetch(url: str, sha: str) -> bytes:
    real_url = url.replace("{sha}", sha)
    print(f"  GET {real_url}", file=sys.stderr)
    req = urllib.request.Request(real_url, headers={"User-Agent": "urlab-sync_vendored"})
    with urllib.request.urlopen(req, timeout=30) as resp:  # nosec: trusted manifest URLs
        return resp.read()


def _maybe_write(path: str, content: bytes, dry_run: bool) -> bool:
    """Returns True if the file content changed."""
    existing = None
    if os.path.exists(path):
        with open(path, "rb") as f:
            existing = f.read()
    if existing == content:
        return False
    if dry_run:
        print(f"  WOULD WRITE {path} ({len(content)} bytes)")
        return True
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "wb") as f:
        f.write(content)
    print(f"  WROTE {path} ({len(content)} bytes; sha256="
          f"{hashlib.sha256(content).hexdigest()[:12]})")
    return True


def main(argv: List[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--manifest", default=_DEFAULT_MANIFEST)
    ap.add_argument("--dry-run", action="store_true",
                    help="don't write any files; print what would happen")
    args = ap.parse_args(argv)

    if not os.path.exists(args.manifest):
        print(f"manifest not found: {args.manifest}", file=sys.stderr)
        return 1

    with open(args.manifest, "r", encoding="utf-8") as f:
        text = f.read()
    preamble = _extract_preamble(text)
    rows = _parse_manifest(text)

    today = _dt.date.today().isoformat()
    updated_rows: List[Dict[str, str]] = []
    for row in rows:
        url = row.get("upstream_url", "")
        sha = row.get("upstream_sha", "")
        dest_rel = row.get("local_dest", "")
        if not (url and sha and dest_rel):
            print(f"skipping malformed row: {row}", file=sys.stderr)
            updated_rows.append(row)
            continue
        try:
            content = _fetch(url, sha)
        except Exception as exc:  # noqa: BLE001
            print(f"  FAILED to fetch {url} @ {sha}: {exc}", file=sys.stderr)
            updated_rows.append(row)
            continue
        dest_abs = os.path.join(_PLUGIN_ROOT, dest_rel.replace("/", os.sep))
        changed = _maybe_write(dest_abs, content, args.dry_run)
        if changed and not args.dry_run:
            row["retrieved_date"] = today
        updated_rows.append(row)

    new_text = _format_manifest(updated_rows, preamble)
    if new_text != text and not args.dry_run:
        with open(args.manifest, "w", encoding="utf-8") as f:
            f.write(new_text)
        print(f"updated manifest: {args.manifest}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
