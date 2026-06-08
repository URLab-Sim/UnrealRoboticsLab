# Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
"""
Tests for the synthetic_categories emitter.

Emits one banner-mode .h file per entry from an mjxmacro X-macro field
list. Covers MjStatistic, MjOption, the 6 MjVisual_* structs, plus
MjsCompilerOptions and MjsSpec.
"""

from __future__ import annotations

import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_CODEGEN_DIR = os.path.dirname(_HERE)
if _CODEGEN_DIR not in sys.path:
    sys.path.insert(0, _CODEGEN_DIR)

from generate_ue_components import (  # noqa: E402
    _emit_synthetic_struct_files,
    _ue_field_name,
)


def test_ue_field_name_pascal_cases_snake_input():
    assert _ue_field_name("timestep") == "Timestep"
    assert _ue_field_name("ls_iterations") == "LsIterations"
    assert _ue_field_name("sleep_tolerance") == "SleepTolerance"
    # Single component stays capitalised.
    assert _ue_field_name("extent") == "Extent"


def test_synthetic_struct_emits_uproperty_per_field():
    mjxmacro = {
        "struct_fields": {
            "TEST_FIELDS": [
                {"type": "mjtNum", "name": "timestep", "count": "1", "is_vec": False},
                {"type": "int",    "name": "iterations", "count": "1", "is_vec": False},
                {"type": "float",  "name": "gravity", "count": "3", "is_vec": True},
            ],
        },
    }
    def_ = {
        "ue_struct_name": "FTestStruct",
        "mjxmacro_block": "TEST_FIELDS",
        "c_struct_type":  "mjTest",
        "public_header":  "MuJoCo/Generated/MjTest.h",
        "private_source": "MuJoCo/Generated/MjTest.cpp",
        "category_label": "Test",
        "exclude_fields": [],
        "per_field_meta": {},
        "urlab_extra_fields": [],
    }
    writes = _emit_synthetic_struct_files(
        "MjTest", def_, mjxmacro, public_root="/tmp/pub", private_root="/tmp/priv",
    )
    assert len(writes) == 1
    h = writes[0].content
    # USTRUCT outline.
    assert "USTRUCT(BlueprintType)" in h
    assert "struct URLAB_API FTestStruct" in h
    # Each field gets its toggle + value pair.
    assert "bOverride_Timestep" in h
    assert "double Timestep" in h
    assert "bOverride_Iterations" in h
    assert "int32 Iterations" in h
    # is_vec + count==3 -> FVector.
    assert "FVector Gravity" in h
    # Category labels use the rule's category_label.
    assert 'Category = "MuJoCo|Test"' in h
    # ApplyTo body assigns each field with a static_cast<decltype(...)>.
    assert "Dst.timestep = static_cast<decltype(Dst.timestep)>(Timestep);" in h
    assert "Dst.gravity[0] = static_cast<decltype(Dst.gravity[0])>(Gravity.X);" in h


def test_synthetic_struct_skips_excluded_fields():
    mjxmacro = {
        "struct_fields": {
            "TEST_FIELDS": [
                {"type": "int", "name": "keep_me", "count": "1", "is_vec": False},
                {"type": "int", "name": "drop_me", "count": "1", "is_vec": False},
            ],
        },
    }
    def_ = {
        "ue_struct_name": "F",
        "mjxmacro_block": "TEST_FIELDS",
        "public_header":  "P/F.h",
        "private_source": "P/F.cpp",
        "category_label": "C",
        "exclude_fields": ["drop_me"],
        "per_field_meta": {},
    }
    writes = _emit_synthetic_struct_files("X", def_, mjxmacro, "/tmp/pub", "/tmp/priv")
    h = writes[0].content
    assert "KeepMe" in h
    assert "DropMe" not in h


def test_synthetic_struct_field_rename_overrides_default():
    """``field_renames`` lets rules pin a non-default UE name (e.g. URLab's
    historical CCD_Iterations vs default CcdIterations)."""
    mjxmacro = {
        "struct_fields": {
            "T_FIELDS": [
                {"type": "int", "name": "ccd_iterations", "count": "1", "is_vec": False},
            ],
        },
    }
    def_ = {
        "ue_struct_name": "FT",
        "mjxmacro_block": "T_FIELDS",
        "public_header":  "P/T.h",
        "private_source": "P/T.cpp",
        "category_label": "T",
        "field_renames":  {"ccd_iterations": "CCD_Iterations"},
    }
    writes = _emit_synthetic_struct_files("MjT", def_, mjxmacro, "/tmp/pub", "/tmp/priv")
    h = writes[0].content
    assert "CCD_Iterations" in h
    assert "CcdIterations" not in h


def test_synthetic_struct_skips_when_mjxmacro_block_missing():
    """Defensive: a typo in mjxmacro_block produces a diagnostic, not a
    crash, and emits no file."""
    from generate_ue_components import _DIAGS_BUFFER, _reset_diags
    _reset_diags()
    mjxmacro = {"struct_fields": {}}
    def_ = {
        "ue_struct_name": "F", "mjxmacro_block": "NONEXISTENT",
        "public_header":  "P/F.h", "private_source": "P/F.cpp",
        "category_label": "C",
    }
    writes = _emit_synthetic_struct_files("M", def_, mjxmacro, "/tmp/pub", "/tmp/priv")
    assert writes == []
    assert any("NONEXISTENT" in d.message for d in _DIAGS_BUFFER.pending)
    _reset_diags()


def test_generated_enum_emits_uenum_per_entry():
    from generate_ue_components import _emit_generated_enum_file
    mjspec = {"enums": {
        "mjtIntegrator": {"mjINT_EULER": 0, "mjINT_RK4": 1, "mjINT_IMPLICIT": 2},
    }}
    def_ = {
        "public_header": "Generated/MjOptionEnums.h",
        "enums": [{
            "ue_name": "EMjIntegrator",
            "from_mj_enum": "mjtIntegrator",
            "ue_member_from_mj": {
                "mjINT_EULER":    "Euler",
                "mjINT_RK4":      "RK4",
                "mjINT_IMPLICIT": "Implicit",
            },
        }],
    }
    writes = _emit_generated_enum_file("MjOptionEnums", def_, mjspec, "/tmp/pub")
    assert len(writes) == 1
    h = writes[0].content
    assert "UENUM(BlueprintType)" in h
    assert "enum class EMjIntegrator : uint8" in h
    assert "Euler = 0" in h
    assert "RK4 = 1" in h
    assert "Implicit = 2" in h


def test_generated_enum_skips_when_mj_enum_missing():
    from generate_ue_components import _emit_generated_enum_file, _DIAGS_BUFFER, _reset_diags
    _reset_diags()
    mjspec = {"enums": {}}
    def_ = {
        "public_header": "P/H.h",
        "enums": [{"ue_name": "E", "from_mj_enum": "mjtMissing", "ue_member_from_mj": {}}],
    }
    writes = _emit_generated_enum_file("X", def_, mjspec, "/tmp/pub")
    assert writes == []
    assert any("mjtMissing" in d.message for d in _DIAGS_BUFFER.pending)
    _reset_diags()


def test_generated_enum_member_without_mapping_silently_skipped():
    """C enums often have sentinel constants (mjNGEOMTYPES) that don't
    map to UE. Rule author omits them from ue_member_from_mj; emitter
    skips them without diagnostics."""
    from generate_ue_components import _emit_generated_enum_file
    mjspec = {"enums": {"mjtX": {"mjX_A": 0, "mjX_SENTINEL": 1, "mjX_B": 2}}}
    def_ = {
        "public_header": "P/H.h",
        "enums": [{
            "ue_name": "EMjX", "from_mj_enum": "mjtX",
            "ue_member_from_mj": {"mjX_A": "A", "mjX_B": "B"},
        }],
    }
    h = _emit_generated_enum_file("X", def_, mjspec, "/tmp/pub")[0].content
    assert "A = 0" in h
    # B retains its original value 2 (skipped sentinel doesn't renumber).
    assert "B = 2" in h
    assert "Sentinel" not in h


def test_real_snapshot_mjstatistic_pilot():
    """End-to-end: the shipped rule produces a syntactically plausible
    .h file. UE compile separately verifies it actually builds."""
    import json
    plugin_root = os.path.normpath(os.path.join(_HERE, "..", "..", ".."))
    h_path = os.path.join(plugin_root, "Source", "URLab", "Public",
                          "MuJoCo", "Generated", "MjStatistic.h")
    assert os.path.exists(h_path), "MjStatistic.h was not generated"
    with open(h_path, "r", encoding="utf-8") as f:
        h = f.read()
    # Pilot fields from MJSTATISTIC_FIELDS.
    for ue_name in ("Meaninertia", "Meanmass", "Meansize", "Extent", "Center"):
        assert ue_name in h, f"missing field {ue_name}"
    # ApplyTo is templated so it works against both mjStatistic and any
    # alias type. Compile correctness is verified by UE's build step.
    assert "template <typename TDst>" in h
    assert "void ApplyTo(TDst& Dst) const" in h
