# Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
"""
Local support modules for URLab codegen.

Currently holds ``ast_nodes.py`` (clang-AST node dataclasses used by
``build_introspect_snapshot.py``). MuJoCo headers are no longer vendored
here — ``mjspecmacro.h`` ships in the installed MuJoCo headers and is read
from ``third_party/install`` like the other headers.
"""
