# Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
"""View-struct emitter tests."""

from __future__ import annotations

from generate_ue_components import emit_view_structs


def test_joint_view_includes_dof_fields(real_rules, real_mjxmacro):
    views = emit_view_structs(real_rules, real_mjxmacro)
    assert "JointView" in views
    fields, bind = views["JointView"]
    # Natural mjModel/JOINT field names (no renames — codegen pure).
    assert "jnt_pos;" in fields
    assert "jnt_axis;" in fields
    assert "jnt_solref;" in fields
    # mjModel/DOF block fields (mapped onto dof_adr).
    assert "dof_damping;" in fields
    assert "dof_solref;" in fields
    # mjData fields.
    assert "qpos;" in fields
    assert "qvel;" in fields
    assert "xanchor;" in fields


def test_joint_view_bind_uses_correct_indices(real_rules, real_mjxmacro):
    views = emit_view_structs(real_rules, real_mjxmacro)
    fields, bind = views["JointView"]
    # Per-joint fields use id stride.
    assert "jnt_pos = m->jnt_pos + id * 3;" in bind
    assert "jnt_axis = m->jnt_axis + id * 3;" in bind
    # solref uses mjNREF stride.
    assert "jnt_solref = m->jnt_solref + id * mjNREF;" in bind
    # DOF fields use dof_adr.
    assert "dof_damping = m->dof_damping + dof_adr * 1;" in bind
    # mjData qpos uses qpos_adr.
    assert "qpos = d->qpos + qpos_adr * 1;" in bind
    # Derived index vars declared at top of bind.
    assert "const int qpos_adr = m->jnt_qposadr[id];" in bind
    assert "const int dof_adr = m->jnt_dofadr[id];" in bind


def test_body_view_basic(real_rules, real_mjxmacro):
    views = emit_view_structs(real_rules, real_mjxmacro)
    assert "BodyView" in views
    fields, bind = views["BodyView"]
    # mjModel/BODY block emits pointer fields.
    # `body_pos`, `body_quat`, `body_mass` should be present (verbatim names).
    assert "body_pos" in fields or "BodyPos" in fields
    # mjData fields present.
    assert "xpos;" in fields
    assert "xquat;" in fields


def test_view_has_one_field_per_mjxmacro_entry(real_rules, real_mjxmacro):
    """Schema-completeness: every entry in the included mjmodel blocks
    should appear as a pointer field on the view."""
    views = emit_view_structs(real_rules, real_mjxmacro)
    fields, _ = views["JointView"]
    joint_block = real_mjxmacro["mjmodel_pointers"]["MJMODEL_POINTERS_JOINT"]
    field_renames = real_rules["views"]["JointView"].get("field_renames", {})
    for entry in joint_block:
        name = entry["name"]
        view_name = field_renames.get(name, name)
        assert f" {view_name};" in fields, f"missing view field for {name} (-> {view_name})"
