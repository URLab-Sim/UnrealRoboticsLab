"""ProtoSpec generator toolchain (build-time Python).

Public surface: the schema front end. ``load_schema`` reads upstream's
``src/xml/mjcf.schema`` through MuJoCo's own parser, applies the overlay, and
returns the AST that all code emitters consume.
"""

from .frontend import OverlayError, SchemaError, load_schema

__all__ = ["load_schema", "OverlayError", "SchemaError"]
