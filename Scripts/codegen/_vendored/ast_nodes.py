# Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
"""
Lightweight AST-node dataclasses used by ``build_introspect_snapshot.py``
to project clang.cindex AST cursors into a serialisable shape.

URLab-side rather than pulled from upstream because (a) the upstream
``clang.cindex`` types don't pickle cleanly across libclang versions and
(b) the snapshot only needs a small subset of node info. Keeping the
projection in one place means the schema is auditable from JSON, and
test fixtures can be hand-built without needing a real libclang install.
"""

from __future__ import annotations

from dataclasses import dataclass, field, asdict
from typing import Any, Dict, List, Optional


@dataclass
class CFunctionParam:
    name:       str
    c_type:     str
    array_dim:  Optional[int] = None       # None for scalar / pointer

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass
class CFunction:
    name:        str
    return_type: str
    params:      List[CFunctionParam] = field(default_factory=list)
    doc:         str = ""
    file:        str = ""                  # source path
    line:        int = 0

    def to_dict(self) -> Dict[str, Any]:
        return {
            "name":        self.name,
            "return_type": self.return_type,
            "params":      [p.to_dict() for p in self.params],
            "doc":         self.doc,
            "file":        self.file,
            "line":        self.line,
        }


@dataclass
class CEnumMember:
    name:  str
    value: int
    doc:   str = ""


@dataclass
class CEnum:
    name:       str
    members:    List[CEnumMember] = field(default_factory=list)
    underlying_type: Optional[str] = None
    doc:        str = ""
    file:       str = ""
    line:       int = 0

    def to_dict(self) -> Dict[str, Any]:
        return {
            "name":            self.name,
            "members":         {m.name: m.value for m in self.members},
            "member_docs":     {m.name: m.doc for m in self.members if m.doc},
            "underlying_type": self.underlying_type,
            "doc":             self.doc,
            "file":            self.file,
            "line":            self.line,
        }


@dataclass
class CStructField:
    name:       str
    c_type:     str
    array_dim:  Optional[int] = None
    is_pointer: bool = False
    doc:        str = ""


@dataclass
class CStruct:
    name:       str
    fields:     List[CStructField] = field(default_factory=list)
    doc:        str = ""
    file:       str = ""
    line:       int = 0

    def to_dict(self) -> Dict[str, Any]:
        return {
            "name":   self.name,
            "fields": [
                {
                    "name":       f.name,
                    "c_type":     f.c_type,
                    "array_dim":  f.array_dim,
                    "is_pointer": f.is_pointer,
                    "doc":        f.doc,
                }
                for f in self.fields
            ],
            "doc":  self.doc,
            "file": self.file,
            "line": self.line,
        }


@dataclass
class CDefine:
    name:  str
    value: str
    doc:   str = ""
    file:  str = ""
    line:  int = 0


@dataclass
class IntrospectSnapshot:
    """Top-level container for the clang-AST scrape output. Serialised
    to ``Scripts/introspect_snapshot.json``. snapshot_version bumps on
    schema-incompatible changes."""
    snapshot_version: int = 1
    header_hash:      str = ""
    functions:        Dict[str, CFunction] = field(default_factory=dict)
    enums:            Dict[str, CEnum] = field(default_factory=dict)
    structs:          Dict[str, CStruct] = field(default_factory=dict)
    defines:          Dict[str, CDefine] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "_meta": {
                "snapshot_version": self.snapshot_version,
                "header_hash":      self.header_hash,
            },
            "functions": {n: f.to_dict() for n, f in self.functions.items()},
            "enums":     {n: e.to_dict() for n, e in self.enums.items()},
            "structs":   {n: s.to_dict() for n, s in self.structs.items()},
            "defines":   {
                n: {"value": d.value, "doc": d.doc, "file": d.file, "line": d.line}
                for n, d in self.defines.items()
            },
        }
