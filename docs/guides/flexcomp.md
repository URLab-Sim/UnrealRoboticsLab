# Flexcomp — Deformable Bodies

Flexcomp is MuJoCo's macro for generating deformable soft bodies: ropes (1D), cloth (2D), and volumetric bodies (3D). URLab's `UMjFlexcomp` component wraps this feature so you can author, import, simulate, and visualize flex bodies inside Unreal.

## What gets generated

A single `<flexcomp>` expands (at compile time) into:

- A `<flex>` element (the actual deformable, with physics properties)
- Multiple child bodies — one per non-pinned vertex (for `dof="full"` / `"radial"`), or 8 corner nodes (for `dof="trilinear"`), or 27 corner nodes (for `dof="quadratic"`)
- Slider joints on each generated body

URLab leverages MuJoCo's own macro expander — we serialize the component's attributes back to an `<flexcomp>` XML fragment, parse it with `mj_parseXMLString`, and attach the resulting sub-spec into the parent body via `mjs_attach`. Every flex feature MuJoCo supports (grid/mesh/direct geometry, pins, elasticity, edge stiffness, contact filtering) works automatically.

## Quick import

The standard drag-and-drop MJCF import pipeline handles `<flexcomp>` natively. Files that reference meshes (e.g. `<flexcomp type="mesh" file="bunny.obj">`) are auto-converted to GLB by the Python preprocessor alongside other mesh assets. The imported Blueprint contains a `UMjFlexcomp` component with all attributes populated and, for `type="mesh"`, a child `UStaticMeshComponent` used as the rendering source.

See the bunny example:
```xml
<flexcomp type="mesh" file="bunny.obj" pos="0 0 .1" dim="2" euler="90 0 0"
          radius=".001" rgba="0 .7 .7 1" mass=".05" name="softbody" dof="trilinear">
    <elasticity young="1e3" poisson="0.1" damping="0.001" elastic2d="stretch"/>
    <contact selfcollide="none" internal="false"/>
</flexcomp>
```

## Authoring in Blueprint

Add a `UMjFlexcomp` component to an `AMjArticulation` Blueprint. Required properties:

| Property | Default | Notes |
|---|---|---|
| `Type` | Grid | grid / box / cylinder / ellipsoid / square / disc / circle / mesh / direct |
| `Dim` | 2 | 1 = lines, 2 = triangles, 3 = tetrahedra |
| `DofType` | Full | full (3 joints/vert), radial (1 joint/vert along radius), trilinear (8 corner bodies), quadratic (27 corner bodies) |
| `Count` / `Spacing` | 10×10×1 / 0.05 | For grid/box/cylinder/ellipsoid types |
| `MeshFile` | *(empty)* | For `type=mesh`, set automatically on import. Also add a child `UStaticMeshComponent` with the mesh asset. |
| `PointData` / `ElementData` | empty | For `type=direct`, raw vertex positions and element indices |

Optional physics/visual properties follow the same `bOverride_X` pattern used throughout the plugin — toggle to override MuJoCo defaults, leave off to inherit. Categories: Contact, Edge, Elasticity, Pin.

## Visualization

At runtime, `UMjFlexcomp::Bind` creates a `UDynamicMeshComponent` mirroring the source static mesh (geometry, UVs, normals copied from LOD 0). Each tick, deformed vertex positions are read from `mjData.flexvert_xpos`, mapped through the welded→raw remap table, and pushed via `FastNotifyPositionsUpdated`. The source static mesh is hidden; its material is reused on the dynamic mesh.

**Known limitation — TAA ghosting.** Because UE's `FLocalVertexFactory` has no previous-position vertex stream and there is no public API to provide one, CPU-deformed meshes produce zero motion vectors. Temporal Anti-Aliasing then accumulates stale previous-frame samples, producing visible smearing/ghost trails on fast deformation. Workarounds:

- Set **Project Settings → Rendering → Anti-Aliasing Method → FXAA** (or `r.AntiAliasingMethod=1` in DefaultEngine.ini) for flex-heavy scenes.
- Use slow, squishy simulations where the effect is subtle.
- A future proper fix requires either a custom vertex factory with a previous-position stream (~1000 LoC + forked shader) or a material-based WPO approach with paired current/previous position textures.

## Supported DOF modes

| DOF | Bodies generated | Use case |
|---|---|---|
| `full` | One body per non-pinned vertex, 3 slider joints each | High-fidelity soft body where every vertex moves independently |
| `radial` | One body per non-pinned vertex, 1 radial slider joint | Inflatable shapes (radial deformation only) |
| `trilinear` | 8 corner bodies, 3 sliders each (24 DOFs total) | Smooth bulk deformation of a whole shape; scales to high vertex counts |
| `quadratic` | 27 corner bodies, 3 sliders each (81 DOFs total) | Higher-order deformation; more detail than trilinear |

Use `trilinear` for meshes with thousands of vertices. `full` DOF on a 2500-vertex bunny creates ~7500 joints, which is slow.

## Known limitations & TODOs

- **TAA ghosting** on the procedural visualization (see above).
- **`bRigid` flexcomps** are attached to the parent body as static surface; they don't deform.
- **Raw `<flex>` elements** (without the `<flexcomp>` macro) aren't directly supported — authors should use `<flexcomp>` to generate them.
- **Per-vertex velocity output** to UE's velocity buffer is not implemented — blocks proper motion blur and TSR on deforming geometry.
