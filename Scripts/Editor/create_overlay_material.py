# Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
# UnrealRoboticsLab is independent and not affiliated with Epic Games. See ThirdPartyNotices.txt.
#
# Headless authoring of the overlay MASTER MATERIAL consumed by UMjOverlayRenderer.
#
# UMjOverlayRenderer writes 4 floats of PerInstanceCustomData per instance -- RGBA
# in slots 0..3 -- on every pooled UInstancedStaticMeshComponent. The engine's
# BasicShapeMaterial ignores that data and is opaque, so translucent/coloured
# overlays (collision hulls, inertia boxes, wrench glyphs, ...) render as flat
# opaque blobs. This script authors /UnrealRoboticsLab/Materials/M_MjOverlay:
#
#   * PerInstanceCustomData[0,1,2] -> Emissive Color (RGB)
#   * PerInstanceCustomData[3]     -> Opacity
#   * BlendMode  = Translucent
#   * ShadingModel = Unlit          (BaseColor is ignored in Unlit; the visible
#                                    colour of an unlit surface is its Emissive,
#                                    so RGB is driven into Emissive Color -- this
#                                    is what makes an unlit overlay show colour)
#   * TwoSided   = true             (thin cylinders/cones read from both sides)
#   * bUsedWithInstancedStaticMeshes = true
#
# RGB is ALSO connected to Base Color so the material still tints correctly if a
# future revision flips it to a lit shading model.
#
# Run headless (UE 5.7):
#   UnrealEditor-Cmd <Project>.uproject -run=pythonscript \
#       -script="<this file>" -unattended -nosplash -nullrhi
# or:
#   UnrealEditor-Cmd <Project>.uproject \
#       -ExecutePythonScript="<this file>" -unattended -nosplash -nullrhi
#
# It creates + saves the .uasset, then this script asks the editor to exit.

import unreal

PACKAGE_PATH = "/UnrealRoboticsLab/Materials"
ASSET_NAME = "M_MjOverlay"
FULL_PATH = "{}/{}".format(PACKAGE_PATH, ASSET_NAME)


def _mk(mat, cls, x, y):
    return unreal.MaterialEditingLibrary.create_material_expression(mat, cls, x, y)


def _per_instance(mat, index, x, y, default):
    node = _mk(mat, unreal.MaterialExpressionPerInstanceCustomData, x, y)
    node.set_editor_property("data_index", index)
    node.set_editor_property("const_default_value", default)
    return node


def create_overlay_material():
    tools = unreal.AssetToolsHelpers.get_asset_tools()

    # Replace any stale asset so re-runs are idempotent.
    if unreal.EditorAssetLibrary.does_asset_exist(FULL_PATH):
        unreal.EditorAssetLibrary.delete_asset(FULL_PATH)

    mat = tools.create_asset(
        ASSET_NAME, PACKAGE_PATH, unreal.Material, unreal.MaterialFactoryNew()
    )
    if mat is None:
        unreal.log_error("[M_MjOverlay] create_asset returned None")
        return False

    # --- material-level settings ---
    mat.set_editor_property("blend_mode", unreal.BlendMode.BLEND_TRANSLUCENT)
    mat.set_editor_property("shading_model", unreal.MaterialShadingModel.MSM_UNLIT)
    mat.set_editor_property("two_sided", True)
    mat.set_editor_property("used_with_instanced_static_meshes", True)

    # --- per-instance custom data -> RGBA ---
    r = _per_instance(mat, 0, -700, -200, 1.0)
    g = _per_instance(mat, 1, -700, -60, 1.0)
    b = _per_instance(mat, 2, -700, 80, 1.0)
    a = _per_instance(mat, 3, -700, 260, 1.0)

    # Append R,G,B into a float3 colour: Append(Append(R,G), B).
    rg = _mk(mat, unreal.MaterialExpressionAppendVector, -450, -120)
    unreal.MaterialEditingLibrary.connect_material_expressions(r, "", rg, "A")
    unreal.MaterialEditingLibrary.connect_material_expressions(g, "", rg, "B")

    rgb = _mk(mat, unreal.MaterialExpressionAppendVector, -250, -40)
    unreal.MaterialEditingLibrary.connect_material_expressions(rg, "", rgb, "A")
    unreal.MaterialEditingLibrary.connect_material_expressions(b, "", rgb, "B")

    # Unlit colour comes from Emissive; also drive Base Color for a possible lit flip.
    unreal.MaterialEditingLibrary.connect_material_property(
        rgb, "", unreal.MaterialProperty.MP_EMISSIVE_COLOR
    )
    unreal.MaterialEditingLibrary.connect_material_property(
        rgb, "", unreal.MaterialProperty.MP_BASE_COLOR
    )
    unreal.MaterialEditingLibrary.connect_material_property(
        a, "", unreal.MaterialProperty.MP_OPACITY
    )

    unreal.MaterialEditingLibrary.recompile_material(mat)

    if not unreal.EditorAssetLibrary.save_asset(FULL_PATH, only_if_is_dirty=False):
        unreal.log_error("[M_MjOverlay] save_asset failed for {}".format(FULL_PATH))
        return False

    unreal.log("[M_MjOverlay] created + saved {}".format(FULL_PATH))
    return True


ok = create_overlay_material()

# Exit the (headless) editor so -run=pythonscript / -ExecutePythonScript terminates
# cleanly with a status the caller can check.
if not ok:
    unreal.log_error("[M_MjOverlay] authoring FAILED")
try:
    unreal.SystemLibrary.quit_editor()
except Exception:
    pass
