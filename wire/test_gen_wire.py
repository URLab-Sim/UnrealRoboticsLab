"""Focused generator regressions for exact tagged operation maps."""

from __future__ import annotations

import copy
import unittest

import gen_wire


class TaggedOperationGenerationTest(unittest.TestCase):
    def test_result_fields_preserve_required_and_optional_flags(self) -> None:
        data = gen_wire.collect(gen_wire._load())
        emitted = gen_wire.emit_cpp(data, "0" * 64)
        self.assertIn(
            '{TEXT("scene_revision"), TEXT("nonempty_string"), nullptr, nullptr, true}',
            emitted,
        )
        self.assertIn(
            '{TEXT("user_camera"), TEXT("render_scene_camera"), nullptr, nullptr, false}',
            emitted,
        )
        self.assertIn(
            '{TEXT("source_time"), TEXT("finite_float"), nullptr, nullptr, false}',
            emitted,
        )

    def test_active_tagged_variants_may_have_disjoint_exact_fields(self) -> None:
        data = gen_wire.collect(gen_wire._load())
        declaration = data["typed_ops"]["data_select"]
        self.assertEqual(declaration["request_shape"], "data_select")
        self.assertEqual(declaration["adapters"], ["zmq"])
        source = data["request_shapes"]["render_source"]["tagged"]["variants"]
        self.assertEqual(set(source), {"mjb", "mjz", "xml", "neutral"})

    def test_active_tagged_variants_reject_conflicting_field_types(self) -> None:
        schema = copy.deepcopy(gen_wire._load())
        schema["protocol_v2_request_shapes"]["data_select_network"]["fields"][
            "region_id"
        ] = {"type": "string", "required": True}
        with self.assertRaisesRegex(ValueError, "tagged field 'region_id' disagrees on type"):
            gen_wire.collect(schema)


if __name__ == "__main__":
    unittest.main()
