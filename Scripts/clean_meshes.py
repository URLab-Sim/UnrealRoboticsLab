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
    python clean_meshes.py <path_to_xml> [--out-dir DIR] [--allow-external-includes]

Without --out-dir the prepared XML is written beside the input. With it, the
prepared XML goes to DIR and every asset path inside is re-expressed so it
still resolves from there, which is how Unreal keeps its prepared copy out of
the model author's own folders.

<include> fragments are flattened into one document on the way through, and an
include whose target is outside the model's own folder is REFUSED unless
--allow-external-includes is passed. That is the same boundary the Unreal
reader enforces, asked here because this step runs first: a prepared import
whose includes were followed by this script would otherwise have its security
answer decided by a script nobody asked.

Exit status is 0 only when every referenced mesh was prepared. A conversion
that failed, a source file that was missing, a refused include, or an
unreadable document exits nonzero with the reason on stderr, because a caller
that continued anyway would import the unprepared meshes at the wrong scale
without saying so.

Example:
    python clean_meshes.py "path/to/mujoco_menagerie/franka_emika_panda/panda.xml"
"""

import trimesh
import numpy as np
from pathlib import Path
from xml.etree import ElementTree as ET
import argparse
import os
import sys
import copy


# MuJoCo's own crease threshold: a face whose normal is more than acos(0.8)
# ~= 36.9 degrees off the vertex normal is not part of that vertex's smooth
# group (mjCMesh::MakeNormal, src/user/user_mesh.cc). Reused here so the
# decision about what counts as an edge matches the simulator, even though the
# representation below is sharper than MuJoCo's own.
CREASE_DOT = 0.8


def split_normals_by_crease(mesh, dot_threshold: float = CREASE_DOT):
    """Split `mesh`'s vertices into smooth groups, so hard edges shade hard.

    The split is geometric rather than a normal override: a vertex whose faces
    disagree by more than the threshold is duplicated, once per group of faces
    that agree. Averaging then happens within a group and never across one, so
    a box corner stays sharp while a finely tessellated cylinder stays round.

    Done this way on purpose. Assigning `mesh.vertex_normals` directly does hold
    in memory, but trimesh recomputes normals during GLB export and the override
    is silently lost -- which looks like a crease split right up until you read
    the accessor back.

    MuJoCo cannot express this at all: it keeps one normal per vertex and merely
    subtracts the outlying contributions, so its corners are a compromise. The
    threshold is still MuJoCo's, so the decision about what counts as an edge
    matches the simulator even though the representation is sharper.
    """
    faces = mesh.faces
    verts = mesh.vertices
    raw = np.cross(verts[faces[:, 1]] - verts[faces[:, 0]],
                   verts[faces[:, 2]] - verts[faces[:, 0]])
    unit = raw / np.maximum(np.linalg.norm(raw, axis=1)[:, None], 1e-20)

    # Faces meeting at each vertex.
    incident = [[] for _ in range(len(verts))]
    for fi, tri in enumerate(faces):
        for vi in tri:
            incident[vi].append(fi)

    new_verts = []
    # (vertex, face) -> index into new_verts
    remap = {}
    for vi, face_ids in enumerate(incident):
        if not face_ids:
            continue
        # Greedy clustering: a face joins the first group whose running mean it
        # agrees with, otherwise it starts one. Enough for the shapes a mesh
        # asset actually has, and it never merges across a real edge.
        groups = []            # list of [summed_normal, [face ids]]
        for fi in face_ids:
            n = unit[fi]
            for g in groups:
                mean = g[0] / max(np.linalg.norm(g[0]), 1e-20)
                if float(np.dot(n, mean)) >= dot_threshold:
                    g[0] = g[0] + n
                    g[1].append(fi)
                    break
            else:
                groups.append([n.copy(), [fi]])
        for g in groups:
            index = len(new_verts)
            new_verts.append(verts[vi])
            for fi in g[1]:
                remap[(vi, fi)] = index

    new_faces = np.empty_like(faces)
    for fi, tri in enumerate(faces):
        for corner, vi in enumerate(tri):
            new_faces[fi, corner] = remap[(vi, fi)]

    # No custom normals: trimesh's own area-weighted average over the split
    # geometry is the crease result, and it survives export because nothing
    # had to be overridden.
    return trimesh.Trimesh(vertices=np.asarray(new_verts), faces=new_faces,
                           process=False)


def clean_mesh(mesh, source_path=None, smooth_normal=False):
    """Clean up a mesh using trimesh."""
    print(f"  Original: {len(mesh.vertices)} vertices, {len(mesh.faces)} faces")

    mesh.merge_vertices(merge_tex=False, merge_norm=False)
    mesh.remove_unreferenced_vertices()
    try:
        import networkx  # noqa: F401 - trimesh's fix_normals requires it
    except ImportError:
        # A GLB exported without fixed normals imports into Unreal with
        # degenerate normals/tangents on every mesh. Failing here makes
        # convert_mesh() skip the GLB entirely, so the importer falls back
        # to the source mesh and Unreal computes clean normals itself.
        raise RuntimeError(
            "networkx is not installed (required by trimesh.fix_normals); "
            "skipping GLB conversion so the source mesh is imported instead. "
            "Install it via the plugin's Python-dependency prompt or "
            "'python -m pip install networkx'."
        )
    mesh.fix_normals()

    # Rotate -90 degrees around X for GLTF Y-up -> Unreal Z-up
    rotation_matrix = trimesh.transformations.rotation_matrix(-np.radians(90), [1, 0, 0])
    mesh.apply_transform(rotation_matrix)

    # Every mesh, not just the ones whose file states no normals. An OBJ that
    # ships split vertices loses them here regardless: the welding above and the
    # GLB export both collapse them, so "the file already answered this" is not
    # a state that survives conversion. Measured on Spot, whose OBJs are flat
    # and whose GLBs came out averaged across every corner.
    #
    # After the transform, never before: applying one invalidates trimesh's
    # cached normals.
    if not smooth_normal:
        mesh = split_normals_by_crease(mesh)
        print(f"  Creased: {len(mesh.vertices)} vertices after splitting hard edges")

    print(f"  Cleaned:  {len(mesh.vertices)} vertices, {len(mesh.faces)} faces")
    return mesh


_SCRIPT_MTIME = Path(__file__).stat().st_mtime


def glb_up_to_date(output_glb: Path, source_path: Path) -> bool:
    """A GLB is stale when older than its source mesh OR older than this
    script -- conversion fixes ship with the script, so GLBs produced by an
    older version must be regenerated."""
    if not output_glb.exists():
        return False
    mtime = output_glb.stat().st_mtime
    return mtime > source_path.stat().st_mtime and mtime > _SCRIPT_MTIME


def convert_mesh(input_path: Path, output_path: Path, smooth_normal: bool = False) -> bool:
    """Convert a single mesh file to GLB."""
    print(f"\n  Converting: {input_path.name} -> {output_path.name}")

    try:
        mesh = trimesh.load(str(input_path), force='mesh')

        if not isinstance(mesh, trimesh.Trimesh):
            print(f"  x Not a valid mesh: {input_path.name}")
            return False

        cleaned_mesh = clean_mesh(mesh, input_path, smooth_normal)

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

        # include_normals: trimesh omits the NORMAL accessor by default and
        # Unreal then builds the mesh with zero normals ("degenerate tangent
        # bases" / "nearly zero normals" on every import).
        cleaned_mesh.export(str(output_path), include_normals=True)
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


class ExternalIncludeError(Exception):
    """An ``<include>`` reaching outside the model's own folder.

    The importer asks before following one of those, because an MJCF document
    can be someone else's and ``<include file="../../../.ssh/id_rsa"/>`` is a
    file read the author of the scene never asked for. This preparation step
    runs BEFORE the reader and flattens the includes itself, so it has to ask
    the same question or the answer the user gave is decided by a script that
    never heard it.
    """

    def __init__(self, include: str, resolved: Path, boundary: Path):
        super().__init__(
            f"refused <include file=\"{include}\"> resolving to {resolved}: "
            f"outside the model's folder {boundary}. Re-run with "
            "--allow-external-includes to allow it.")
        self.include = include
        self.resolved = resolved
        self.boundary = boundary


def _inside(path: Path, boundary: Path) -> bool:
    """True when ``path`` is ``boundary`` or below it, symlinks resolved."""
    try:
        return path.resolve().is_relative_to(boundary.resolve())
    except (OSError, ValueError):
        # A path on another drive, or one the OS will not resolve, is outside
        # by definition; failing closed is the whole point of the check.
        return False


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
                     meshdir: str, texturedir: str, assetdir: str, visited: set,
                     boundary: Path = None):
    """Copy ``elem`` into ``out_parent``, recursively expanding any <include>
    descendants in place. Handles both <mujoco> and <mujocoinclude> include roots
    and rewrites asset file= paths to stay valid from root_dir.

    ``boundary``, when given, is the directory tree an include may not leave;
    one that does raises ExternalIncludeError naming it."""
    if elem.tag == "include":
        inc_file = elem.get("file")
        if not inc_file:
            return
        inc_path = (src_dir / inc_file).resolve()
        if boundary is not None and not _inside(inc_path, boundary):
            raise ExternalIncludeError(inc_file, inc_path, boundary)
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
            _append_expanded(out_parent, child, inc_dir, root_dir, imd, itxd, iad,
                             visited, boundary)
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
                         meshdir, texturedir, assetdir, visited, boundary)


def flatten_includes(root, src_dir: Path, root_dir: Path = None,
                     boundary: Path = None):
    """Resolve every <include> into a single self-contained <mujoco> tree.

    gym-aloha (and many MJCF models) split a robot across <include> fragments
    rooted in <mujocoinclude>, including files referenced from inside <worldbody>.
    Unreal's importer must see one flat document so all <asset>/<compiler>/body
    content is visible and there is exactly one worldbody. Returns the new root
    element (a <mujoco>). No-op-equivalent for files without includes.

    ``src_dir`` is where the document being flattened lives; ``root_dir`` is
    where the flattened document will be written, and defaults to ``src_dir``.
    ``boundary`` is the directory tree includes may not leave, or None to follow
    them anywhere.
    """
    if root_dir is None:
        root_dir = src_dir
    md, txd, ad = _compiler_dirs(root)
    visited = set()
    new_root = ET.Element("mujoco", dict(root.attrib))
    for child in list(root):
        _append_expanded(new_root, child, src_dir, root_dir, md, txd, ad, visited,
                         boundary)
    return new_root


def _rebase_assets(root, src_dir: Path, root_dir: Path):
    """Re-express every asset ``file=`` so it resolves from ``root_dir``.

    What ``flatten_includes`` does on the way through, for a document that has
    no includes to flatten. The <compiler> dir attributes are cleared once the
    paths carry their own prefixes, so nothing double-prefixes them.
    """
    md, txd, ad = _compiler_dirs(root)
    for elem in root.iter():
        if elem.tag in _ASSET_FILE_TAGS:
            _rewrite_asset_path(elem, src_dir, root_dir, md, txd, ad)
    for comp in root.findall("compiler"):
        for attr in ("meshdir", "texturedir", "assetdir"):
            comp.attrib.pop(attr, None)


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


def process_xml(xml_path: Path, out_dir: Path = None,
                allow_external_includes: bool = False) -> bool:
    """Parse MJCF XML, convert meshes, resolve conflicts, write updated XML.

    Returns True only when every mesh the document references was prepared, and
    only when every ``<include>`` stayed inside the model's own folder unless
    ``allow_external_includes`` says otherwise.
    """

    if not xml_path.exists():
        print(f"Error: XML file not found: {xml_path}", file=sys.stderr)
        return False

    xml_dir = xml_path.parent.resolve()
    root_dir = out_dir.resolve() if out_dir is not None else xml_dir
    root_dir.mkdir(parents=True, exist_ok=True)

    print(f"XML: {xml_path}")
    print(f"Dir: {xml_dir}")
    if root_dir != xml_dir:
        print(f"Out: {root_dir}")
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
        # The model's own folder is the boundary, matching the reader's rule.
        # Off only when the caller passed the same option the import dialog
        # shows, so the gate is decided in one place rather than twice.
        boundary = None if allow_external_includes else xml_dir
        try:
            root = flatten_includes(root, xml_dir, root_dir, boundary)
        except ExternalIncludeError as refused:
            print(f"Error: {refused}", file=sys.stderr)
            return False
        tree = ET.ElementTree(root)
        remaining = sum(1 for _ in root.iter("include"))
        print(f"  -> {remaining} include(s) remain after flatten")
    elif root_dir != xml_dir:
        _rebase_assets(root, xml_dir, root_dir)

    # Find meshdir from compiler
    meshdir = ""
    for compiler in root.iter("compiler"):
        md = compiler.get("meshdir", "")
        if md:
            meshdir = md

    mesh_base = root_dir / meshdir if meshdir else root_dir
    print(f"Mesh directory: {mesh_base}")

    # Every mesh this run could not prepare. Non-empty means the caller must
    # not use the output: the unconverted source would import at its own scale.
    failures = []

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
                if not glb_up_to_date(output_glb, source_path):
                    print(f"\n[flexcomp] Converting mesh: {source_path.name} -> {output_glb.name}")
                    if not convert_mesh(source_path, output_glb):
                        failures.append(f"flexcomp mesh '{file_attr}' failed to convert")
                        if output_glb.exists():
                            output_glb.unlink()
                            print(f"[flexcomp] Removed stale GLB: {output_glb.name}")
                else:
                    print(f"\n[flexcomp] Mesh up to date: {output_glb.name}")
            else:
                failures.append(f"flexcomp mesh not found: {source_path}")

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
                        print(f"    x Source file missing: {source_path}")
                        failures.append(f"mesh source not found: {source_path}")
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
            failures.append(f"mesh source not found: {actual_source}")
            continue

        if glb_up_to_date(output_glb, actual_source):
            print(f"\n  Skipping '{mesh_name}' (GLB up to date): {output_glb.name}")
            success_count += 1
            continue

        print(f"\n[{mesh_name}] {actual_source.name} -> {output_glb.name}")
        # MJCF's own opt-in to one averaged normal per vertex. It defaults to
        # false, so a model that says nothing gets the crease split.
        smooth_normal = mesh_el.get("smoothnormal", "false").strip().lower() in ("true", "1")

        if convert_mesh(actual_source, output_glb, smooth_normal):
            success_count += 1
        else:
            print(f"  x FAILED to convert {actual_source.name}")
            failures.append(f"mesh '{mesh_name}' failed to convert: {actual_source}")
            # Never leave a stale or partial GLB behind: the importer
            # prefers .glb over the source mesh, so a leftover here would
            # silently ship the very data the conversion just refused to
            # produce.
            if output_glb.exists():
                output_glb.unlink()
                print(f"  -> Removed stale GLB: {output_glb.name}")

    # Phase 4: Write updated XML. Written even when something failed, so the
    # partial result can be inspected; the exit status is what decides whether
    # a caller may use it.
    output_xml = root_dir / f"{xml_path.stem}_ue.xml"
    tree.write(str(output_xml), encoding="unicode", xml_declaration=True)

    print("\n" + "=" * 60)
    print(f"Processed {success_count}/{len(output_plan)} meshes successfully")
    print(f"Conflicts resolved: {conflicts_found}")
    print(f"Updated XML: {output_xml}")

    if failures:
        print(f"\n{len(failures)} mesh(es) could not be prepared:", file=sys.stderr)
        for reason in failures:
            print(f"  - {reason}", file=sys.stderr)
        return False

    print(f"\nDrag '{output_xml.name}' into Unreal Content Browser to import.")
    return True


def main():
    parser = argparse.ArgumentParser(
        description="Prepare MuJoCo MJCF meshes for Unreal import.")
    parser.add_argument("xml", type=Path, help="path to the MJCF .xml to prepare")
    parser.add_argument("--out-dir", type=Path, default=None, dest="out_dir",
                        help="directory to write the prepared _ue.xml into "
                             "(default: beside the input)")
    parser.add_argument("--allow-external-includes", action="store_true",
                        dest="allow_external_includes",
                        help="follow an <include> whose target is outside the "
                             "model's own folder (refused by default)")
    args = parser.parse_args()

    if args.xml.suffix.lower() != ".xml":
        parser.error(f"expected an .xml file, got '{args.xml.suffix}'")

    return 0 if process_xml(args.xml, args.out_dir,
                            args.allow_external_includes) else 1


if __name__ == "__main__":
    sys.exit(main())
