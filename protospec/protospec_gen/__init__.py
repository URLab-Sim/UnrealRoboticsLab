"""ProtoSpec generator toolchain (build-time Python).

Public surface: the schema front end. ``load_schema`` reads upstream's
``src/xml/mjcf.schema`` through MuJoCo's own parser, applies the overlay, and
returns the AST that all code emitters consume.

``SchemaError`` is upstream's failure type, re-exported. It is resolved on first
use rather than on import, because naming it is what makes the parser load and
importing this package must not require a MuJoCo checkout.
"""

from .frontend import OverlayError, load_schema

__all__ = ["load_schema", "OverlayError", "SchemaError"]


def __getattr__(name):
    if name == "SchemaError":
        from . import frontend

        return frontend.SchemaError
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
