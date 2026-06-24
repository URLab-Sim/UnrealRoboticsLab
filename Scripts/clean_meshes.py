# Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# --- LEGAL DISCLAIMER ---
# UnrealRoboticsLab is an independent software plugin. It is NOT affiliated with,
# endorsed by, or sponsored by Epic Games, Inc. "Unreal" and "Unreal Engine" are
# trademarks or registered trademarks of Epic Games, Inc. in the US and elsewhere.
#
# This plugin incorporates third-party software: MuJoCo (Apache 2.0),
# CoACD (MIT), and libzmq (MPL 2.0). See ThirdPartyNotices.txt for details.

"""
Prepare MuJoCo MJCF meshes for Unreal import.

Parses an MJCF XML, converts all referenced meshes (OBJ, STL) to GLB,
resolves naming conflicts (e.g., link1.obj and link1.stl both becoming link1.glb),
and writes an updated XML ready for drag-and-drop import into Unreal.

Installation:
    pip install trimesh numpy scipy Pillow networkx

Usage:
    python clean_meshes_trimesh.py <path_to_xml>

Example:
    python clean_meshes_trimesh.py "path/to/mujoco_menagerie/franka_emika_panda/panda.xml"
"""

import trimesh
import numpy as np
from pathlib import Path
from xml.etree import ElementTree as ET
import os
import sys
import copy


def clean_mesh(mesh):
    """Clean up a mesh using trimesh."""
    print(f"  Original: {len(mesh.vertices)} vertices, {len(mesh.faces)} faces")

    mesh.merge_vertices(merge_tex=False, merge_norm=False)
    mesh.remove_unreferenced_vertices()
    try:
        import networkx  # noqa: F401 - trimesh's fix_normals requires it
        mesh.fix_normals()
    except ImportError:
        print("  Warning: networkx not installed, skipping fix_normals (GLB may have lighting issues)")

    # Rotate -90 degrees around X for GLTF Y-up -> Unreal Z-up
    rotation_matrix = trimesh.transformations.rotation_matrix(-np.radians(90), [1, 0, 0])
    mesh.apply_transform(rotation_matrix)

    print(f"  Cleaned:  {len(mesh.vertices)} vertices, {len(mesh.faces)} faces")
    return mesh


def convert_mesh(input_path: Path, output_path: Path) -> bool:
    """Convert a single mesh file to GLB."""
    print(f"\n  Converting: {input_path.name} -> {output_path.name}")

    try:
        mesh = trimesh.load(str(input_path), force='mesh')

        if not isinstance(mesh, trimesh.Trimesh):
            print(f"  x Not a valid mesh: {input_path.name}")
            return False

        cleaned_mesh = clean_mesh(mesh)

        # Strip embedded materials/textures to prevent Unreal's Interchange importer
        # from creating a Texture2D instead of a StaticMesh.
        # Preserve UV coordinates so textures can be applied via material instances.
        if hasattr(cleaned_mesh.visual, 'uv') and cleaned_mesh.visual.uv is not None:
            uv = cleaned_mesh.visual.uv.copy()
            cleaned_mesh.visual = trimesh.visual.TextureVisuals(uv=uv)
        else:
            cleaned_mesh.visual = trimesh.visual.ColorVisuals()

        if len(cleaned_mesh.vertices) == 0 or len(cleaned_mesh.faces) == 0:
            print(f"  x Mesh became empty after cleaning")
            return False

        bounds = cleaned_mesh.bounds
        size = bounds[1] - bounds[0]
        print(f"  Bounds: {size}")

        if np.allclose(size, 0):
            print(f"  Warning: Mesh has zero size!")

        cleaned_mesh.export(str(output_path))
        print(f"  -> Saved: {output_path.name}")
        return True

    except Exception as e:
        print(f"  x Error: {e}")
        return False


def _parse_floats(s: str) -> list:
    """Whitespace-tolerant float list parser for inline mesh attributes."""
    return [float(t) for t in s.replace(",", " ").split() if t]


# Asset elements whose ``file=`` attribute is a path that must stay resolvable
# after includes are flattened into a single file in a (possibly) different dir.
_ASSET_FILE_TAGS = ("mesh", "texture", "hfield", "skin")
# Tags that may legally be the root of an MJCF document or an include fragment.
_MODEL_ROOT_TAGS = ("mujoco", "mujocoinclude")


def _compiler_dirs(root) -> tuple:
    """Return (meshdir, texturedir, assetdir) declared by any <compiler> directly
    under ``root``. MuJoCo allows several compiler elements; later ones win for a
    given attribute, matching the C++ importer's last-writer behaviour."""
    md = txd = ad = ""
    for comp in root.findall("compiler"):
        md = comp.get("meshdir", md)
        txd = comp.get("texturedir", txd)
        ad = comp.get("assetdir", ad)
    return md, txd, ad


def _rewrite_asset_path(elem, src_dir: Path, root_dir: Path,
                        meshdir: str, texturedir: str, assetdir: str):
    """Rewrite an asset element's ``file=`` so it resolves from ``root_dir`` (the
    output _ue.xml location) after the declaring file has been spliced in.

    Paths are resolved relative to the *declaring* file's directory plus its
    meshdir/texturedir/assetdir, matching how the C++ importer resolves includes,
    then re-expressed relative to root_dir. The companion <compiler> dir
    attributes are cleared by the caller so nothing double-prefixes them.
    """
    file_attr = elem.get("file")
    if not file_attr:
        return
    if elem.tag == "texture":
        subdir = texturedir or assetdir
    else:  # mesh, hfield, skin
        subdir = meshdir or assetdir
    base = src_dir / subdir if subdir else src_dir
    abs_path = (base / file_attr).resolve()
    try:
        rel = os.path.relpath(abs_path, root_dir)
    except ValueError:
        rel = str(abs_path)  # different drive on Windows — keep absolute
    elem.set("file", rel.replace("\\", "/"))


def _append_expanded(out_parent, elem, src_dir: Path, root_dir: Path,
                     meshdir: str, texturedir: str, assetdir: str, visited: set):
    """Copy ``elem`` into ``out_parent``, recursively expanding any <include>
    descendants in place. Handles both <mujoco> and <mujocoinclude> include roots
    and rewrites asset file= paths to stay valid from root_dir."""
    if elem.tag == "include":
        inc_file = elem.get("file")
        if not inc_file:
            return
        inc_path = (src_dir / inc_file).resolve()
        if inc_path in visited:
            print(f"  [include] cycle/duplicate skipped: {inc_path}")
            return
        if not inc_path.exists():
            print(f"  x [include] file not found, leaving unresolved: {inc_path}")
            return
        visited.add(inc_path)
        inc_root = ET.parse(str(inc_path)).getroot()
        inc_dir = inc_path.parent
        imd, itxd, iad = _compiler_dirs(inc_root)
        # Splice the included root's children directly into the current parent.
        for child in list(inc_root):
            _append_expanded(out_parent, child, inc_dir, root_dir, imd, itxd, iad, visited)
        return

    # Regular element: shallow-copy attributes, then recurse into children.
    new_elem = ET.SubElement(out_parent, elem.tag, dict(elem.attrib))
    new_elem.text = elem.text
    new_elem.tail = elem.tail

    if elem.tag in _ASSET_FILE_TAGS:
        _rewrite_asset_path(new_elem, src_dir, root_dir, meshdir, texturedir, assetdir)
    elif elem.tag == "compiler":
        # Paths are now baked into each file=, so clear the dir prefixes.
        for attr in ("meshdir", "texturedir", "assetdir"):
            new_elem.attrib.pop(attr, None)

    for child in list(elem):
        _append_expanded(new_elem, child, src_dir, root_dir,
                         meshdir, texturedir, assetdir, visited)


def flatten_includes(root, root_dir: Path):
    """Resolve every <include> into a single self-contained <mujoco> tree.

    gym-aloha (and many MJCF models) split a robot across <include> fragments
    rooted in <mujocoinclude>, including files referenced from inside <worldbody>.
    Unreal's importer must see one flat document so all <asset>/<compiler>/body
    content is visible and there is exactly one worldbody. Returns the new root
    element (a <mujoco>). No-op-equivalent for files without includes.
    """
    md, txd, ad = _compiler_dirs(root)
    visited = set()
    new_root = ET.Element("mujoco", dict(root.attrib))
    for child in list(root):
        _append_expanded(new_root, child, root_dir, root_dir, md, txd, ad, visited)
    return new_root


def materialize_inline_meshes(root, mesh_base: Path) -> int:
    """Rewrite ``<mesh vertex="..." face="...">`` (MuJoCo's inline-data form)
    to ``<mesh file="...">`` by emitting an OBJ next to the rest of the
    mesh assets. Returns the number of inline meshes materialized.

    MuJoCo accepts meshes declared inline via ``vertex`` (flat list of
    3-tuple floats) and ``face`` (flat list of 3-tuple ints) attributes
    instead of an external ``file``. Unreal's importer can't follow the
    inline form, so we synthesize a real OBJ on disk and rewrite the
    ``<mesh>`` element to point at it. The downstream conversion phase
    then produces a GLB companion just like any other ``file=`` mesh.
    """
    materialized = 0
    mesh_base.mkdir(parents=True, exist_ok=True)
    for mesh_el in list(root.iter("mesh")):
        if mesh_el.get("file"):
            continue
        vertex_attr = mesh_el.get("vertex")
        face_attr = mesh_el.get("face")
        if not vertex_attr or not face_attr:
            continue

        name = mesh_el.get("name")
        if not name:
            print(f"  x Inline mesh has no name; skipping (cannot synthesize filename)")
            continue

        try:
            verts = _parse_floats(vertex_attr)
            faces = [int(t) for t in face_attr.replace(",", " ").split() if t]
        except ValueError as e:
            print(f"  x Failed to parse vertex/face for '{name}': {e}")
            continue

        if len(verts) % 3 != 0 or len(faces) % 3 != 0:
            print(f"  x Inline mesh '{name}' has non-triangular layout "
                  f"(verts={len(verts)}, faces={len(faces)}) — skipping")
            continue

        n_verts = len(verts) // 3
        n_faces = len(faces) // 3

        # Filename-safe stem. MuJoCo names can contain '/' (the scene XML
        # uses slash-paths heavily) which would create stray directories.
        safe_stem = name.replace("/", "_").replace("\\", "_").replace(":", "_")
        obj_path = mesh_base / f"{safe_stem}.obj"

        print(f"\n[inline] Materializing '{name}' -> {obj_path.name} "
              f"({n_verts} verts, {n_faces} faces)")

        # Standard OBJ — vertices are 1-indexed in OBJ but 0-indexed in MuJoCo.
        with open(obj_path, "w", encoding="utf-8") as f:
            f.write(f"# Generated by clean_meshes.py from inline MJCF mesh '{name}'\n")
            for i in range(n_verts):
                f.write(f"v {verts[i*3]:.6f} {verts[i*3+1]:.6f} {verts[i*3+2]:.6f}\n")
            for i in range(n_faces):
                a, b, c = faces[i*3] + 1, faces[i*3+1] + 1, faces[i*3+2] + 1
                f.write(f"f {a} {b} {c}\n")

        # Rewrite the element: drop inline data, point at the new OBJ.
        # Path is relative to meshdir (matching how file= entries already
        # work). We preserve scale and content_type if present.
        rel_path = obj_path.name
        del mesh_el.attrib["vertex"]
        del mesh_el.attrib["face"]
        mesh_el.set("file", rel_path)
        if "content_type" not in mesh_el.attrib:
            mesh_el.set("content_type", "model/obj")
        materialized += 1

    return materialized


def process_xml(xml_path: Path):
    """Parse MJCF XML, convert meshes, resolve conflicts, write updated XML."""

    if not xml_path.exists():
        print(f"Error: XML file not found: {xml_path}")
        return

    xml_dir = xml_path.parent

    print(f"XML: {xml_path}")
    print(f"Dir: {xml_dir}")
    print("=" * 60)

    # Parse XML
    tree = ET.parse(str(xml_path))
    root = tree.getroot()

    # Phase -1: Flatten <include> fragments (gym-aloha and friends split robots
    # across <mujocoinclude> files, some referenced from inside <worldbody>).
    # After this the tree is a single self-contained <mujoco> with no includes,
    # so mesh conversion below sees every mesh and Unreal imports one flat model.
    include_count = sum(1 for _ in root.iter("include"))
    if include_count:
        print(f"Flattening {include_count} <include> fragment(s)...")
        root = flatten_includes(root, xml_dir.resolve())
        tree = ET.ElementTree(root)
        remaining = sum(1 for _ in root.iter("include"))
        print(f"  -> {remaining} include(s) remain after flatten")

    # Find meshdir from compiler
    meshdir = ""
    for compiler in root.iter("compiler"):
        md = compiler.get("meshdir", "")
        if md:
            meshdir = md

    mesh_base = xml_dir / meshdir if meshdir else xml_dir
    print(f"Mesh directory: {mesh_base}")

    # Phase 0: Materialize inline ``<mesh vertex="..." face="...">`` entries.
    # These get rewritten to file= entries before Phase 1 runs, so the rest
    # of the pipeline treats them like any other on-disk mesh.
    inline_count = materialize_inline_meshes(root, mesh_base)
    if inline_count:
        print(f"Materialized {inline_count} inline mesh(es) to {mesh_base}")

    # Collect all mesh elements
    mesh_elements = list(root.iter("mesh"))

    # Also convert meshes referenced by <flexcomp file="...">
    for flexcomp in root.iter("flexcomp"):
        file_attr = flexcomp.get("file")
        if file_attr:
            source_path = mesh_base / file_attr
            if source_path.exists():
                output_glb = source_path.with_suffix(".glb")
                if not output_glb.exists() or output_glb.stat().st_mtime < source_path.stat().st_mtime:
                    print(f"\n[flexcomp] Converting mesh: {source_path.name} -> {output_glb.name}")
                    convert_mesh(source_path, output_glb)
                else:
                    print(f"\n[flexcomp] Mesh up to date: {output_glb.name}")

    print(f"Found {len(mesh_elements)} mesh assets in XML\n")

    # Phase 1: Plan output filenames, detect conflicts
    # Map: glb_stem -> list of (mesh_element, source_path)
    stem_usage = {}

    for mesh_el in mesh_elements:
        file_attr = mesh_el.get("file")
        if not file_attr:
            continue

        source_path = mesh_base / file_attr
        glb_stem = source_path.stem  # e.g., "link1" from "link1.stl"

        # Ensure explicit name attribute exists — MuJoCo defaults to filename stem
        # if omitted. We must preserve it before changing the file attribute.
        if mesh_el.get("name") is None:
            implicit_name = Path(file_attr).stem
            mesh_el.set("name", implicit_name)
            print(f"  Set explicit name='{implicit_name}' on mesh with file='{file_attr}'")

        if glb_stem not in stem_usage:
            stem_usage[glb_stem] = []
        stem_usage[glb_stem].append((mesh_el, source_path))

    # Phase 2: Assign unique output names and resolve conflicts
    # For each mesh: keep the original format (OBJ/STL) in the XML (MuJoCo needs it),
    # but rename conflicting files so their stems are unique. Then convert each to GLB
    # alongside — Unreal's importer will find the GLB via "higher priority" fallback.
    #
    # output_plan: list of (mesh_element, source_path, renamed_source_path, output_glb_path, new_file_attr)
    import shutil
    output_plan = []
    conflicts_found = 0

    for glb_stem, entries in stem_usage.items():
        if len(entries) == 1:
            # No conflict
            mesh_el, source_path = entries[0]
            output_glb = source_path.with_suffix(".glb")
            # file attr stays unchanged in the XML
            output_plan.append((mesh_el, source_path, source_path, output_glb))
        else:
            # Conflict — multiple source files share the same stem
            conflicts_found += len(entries) - 1
            print(f"  CONFLICT: {len(entries)} files map to stem '{glb_stem}':")
            for i, (mesh_el, source_path) in enumerate(entries):
                mesh_name = mesh_el.get("name", "?")
                print(f"    [{i}] mesh name='{mesh_name}' <- {source_path.name}")

            # First one keeps original name, rest get a counter suffix
            for i, (mesh_el, source_path) in enumerate(entries):
                if i == 0:
                    # No rename needed
                    output_glb = source_path.with_suffix(".glb")
                    output_plan.append((mesh_el, source_path, source_path, output_glb))
                else:
                    # Rename: link1.stl -> link1_1.stl, link1_1.glb
                    # Save to meshdir root (not the source subdirectory) so the
                    # XML file attribute stays a simple filename relative to meshdir.
                    new_stem = f"{glb_stem}_{i}"
                    ext = source_path.suffix  # .stl, .obj, etc.
                    renamed_source = mesh_base / f"{new_stem}{ext}"
                    output_glb = mesh_base / f"{new_stem}.glb"

                    # Update XML file attribute (relative to meshdir)
                    new_file_attr = f"{new_stem}{ext}"
                    mesh_el.set("file", new_file_attr)

                    mesh_name = mesh_el.get("name", "?")
                    print(f"    -> Renaming '{mesh_name}': {source_path.name} -> {new_stem}{ext}")

                    # Copy the source file to the new name
                    if not source_path.exists():
                        print(f"    x Source file missing: {source_path} — skipping")
                        continue
                    if not renamed_source.exists() or renamed_source.stat().st_mtime < source_path.stat().st_mtime:
                        shutil.copy2(str(source_path), str(renamed_source))
                        print(f"    -> Copied {source_path.name} -> {renamed_source.name}")

                    output_plan.append((mesh_el, source_path, renamed_source, output_glb))

    print(f"\n{len(output_plan)} meshes to convert, {conflicts_found} naming conflicts resolved")
    print("=" * 60)

    # Phase 3: Convert meshes to GLB (alongside the originals)
    success_count = 0
    for mesh_el, original_source, actual_source, output_glb in output_plan:
        mesh_name = mesh_el.get("name", "?")

        if not actual_source.exists():
            print(f"\n  x Source not found: {actual_source}")
            continue

        # Skip if GLB already exists and is newer than source
        if output_glb.exists() and output_glb.stat().st_mtime > actual_source.stat().st_mtime:
            print(f"\n  Skipping '{mesh_name}' (GLB up to date): {output_glb.name}")
            success_count += 1
            continue

        print(f"\n[{mesh_name}] {actual_source.name} -> {output_glb.name}")
        if convert_mesh(actual_source, output_glb):
            success_count += 1
        else:
            print(f"  x FAILED to convert {actual_source.name}")

    # Phase 4: Write updated XML
    output_xml = xml_path.parent / f"{xml_path.stem}_ue.xml"
    tree.write(str(output_xml), encoding="unicode", xml_declaration=True)

    print("\n" + "=" * 60)
    print(f"Processed {success_count}/{len(output_plan)} meshes successfully")
    print(f"Conflicts resolved: {conflicts_found}")
    print(f"Updated XML: {output_xml}")
    print(f"\nDrag '{output_xml.name}' into Unreal Content Browser to import.")


def main():
    if len(sys.argv) < 2:
        print("Error: No XML file specified")
        print("Usage: python clean_meshes_trimesh.py <path_to_xml>")
        print('Example: python clean_meshes_trimesh.py "C:/mujoco_menagerie/franka_emika_panda/panda.xml"')
        return

    xml_path = Path(sys.argv[1])

    if xml_path.suffix.lower() == ".xml":
        process_xml(xml_path)
    else:
        print(f"Error: Expected an .xml file, got '{xml_path.suffix}'")
        print("Usage: python clean_meshes_trimesh.py <path_to_xml>")


if __name__ == "__main__":
    main()
