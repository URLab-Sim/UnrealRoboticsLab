# Geometry & Collision

Define collision shapes, from analytic primitives to decomposed meshes, control which bodies collide, and add deformable soft bodies.

![Collision wireframes overlaid on a robot showing primitive and mesh geoms](../images/placeholder.svg)

## Collision shapes

Add geoms as children of a body. Box, sphere, and cylinder primitives (`MjBox`, `MjSphere`, `MjCylinder`) give fast, exact analytic collision, and you can stack several on one body to build a compound shape. Size them with the standard Unreal transform gizmo; URLab reads the component scale and maps it to MuJoCo size parameters. Capsules are available on `MjGeom` with the type set to Capsule.

For a mesh geom, set the type to Mesh and assign a Static Mesh asset. MuJoCo treats it as convex and computes the convex hull internally. For a **concave** mesh, decompose it: click **Decompose Mesh** on the geom (or set `ComplexMeshRequired` on a Quick Convert component), which runs CoACD to produce a set of convex pieces.

For terrain, `AMjHeightfieldActor` turns Unreal Landscape geometry into a MuJoCo heightfield. Enable `bTraceComplex` for accurate mesh-level sampling, and use `TraceWhitelist` to limit which actors are sampled (for example terrain only, not foliage).

## Quick Convert (props from static meshes)

`UMjQuickConvertComponent` turns any Static Mesh actor into a physics body in one step. Add it to the actor and configure:

- **Static** keeps the body fixed in place (no free joint).
- **ComplexMeshRequired** runs CoACD decomposition for concave meshes; **CoACDThreshold** controls detail (0.01 to 1.0, lower gives more convex pieces).
- **bDrivenByUnreal** lets Unreal drive the transform while MuJoCo does not simulate it.

This is the fastest way to add furniture, obstacles, and throwable props to a scene.

## Geom properties

Beyond shape, each geom carries the usual MuJoCo physical properties: `Friction` (sliding, torsional, rolling), `Density` (mass is inferred from density times volume), `Margin` (contact detection margin), and `Solref` / `Solimp` (solver stiffness, damping, and impedance).

For explicit mass instead of density-inferred mass, add a `UMjInertial` to the body with a specific mass, center of mass, and inertia matrix.

## Contact filtering

Control which geoms can collide:

- **Bitmasks** (`Contype` / `Conaffinity`): two geoms generate contact only if their type and affinity masks overlap. This scales well for large scenes.
- **Contact pairs** (`UMjContactPair`): explicitly enable contact between two specific geoms.
- **Contact exclusions** (`UMjContactExclude`): disable contact between two specific bodies.

Groups (0 to 5) are integer tags for organising visibility, not contact generation. By convention URLab uses group 2 for visual-only meshes (`contype=0`, `conaffinity=0`) and group 3 for collision-only meshes.

!!! tip
    The collision debug overlay draws geoms in group 3 or with both `contype` and `conaffinity` non-zero, so contact spheres still show even when they are not in group 3. Press **2** during PIE to hide visual meshes and inspect collision shapes alone. See [Debug Visualization](debug.md).

## Deformable bodies (flexcomp)

Flexcomp is MuJoCo's macro for soft bodies: ropes (1D), cloth (2D), and volumetric solids (3D). URLab's `UMjFlexcomp` component wraps it so you can import, author, simulate, and visualise deformables.

The standard drag-and-drop import handles `<flexcomp>` natively, including mesh-backed soft bodies (the referenced mesh is auto-converted to GLB like any other). See [Importing Robots](importing.md). To author one by hand, add a `UMjFlexcomp` to an articulation and set its key properties:

| Property | Notes |
|---|---|
| `Type` | grid, box, cylinder, ellipsoid, square, disc, circle, mesh, or direct |
| `Dim` | 1 = lines, 2 = triangles, 3 = tetrahedra |
| `DofType` | how the deformable is driven (see below) |
| `Count` / `Spacing` | resolution for the generated shape types |
| `MeshFile` | for `type=mesh`; pair with a child `UStaticMeshComponent` holding the mesh |

Optional contact, edge, elasticity, and pin properties follow the plugin's usual `bOverride_X` pattern: toggle to override a MuJoCo default, leave it off to inherit.

**Choosing a DOF type.** `full` gives every vertex independent motion (highest fidelity, most joints), `radial` allows radial-only motion (inflatables), `trilinear` drives the whole shape through 8 corner bodies, and `quadratic` uses 27 corner bodies for higher-order deformation. Prefer `trilinear` for meshes with thousands of vertices: `full` on a 2500-vertex mesh creates roughly 7500 joints and runs slowly.

At runtime the component builds a dynamic mesh that mirrors the source geometry and updates vertex positions from the simulation each tick.

!!! warning
    CPU-deformed meshes produce no motion vectors, so Temporal Anti-Aliasing accumulates stale samples and smears fast deformation. For flex-heavy scenes, set **Project Settings, Rendering, Anti-Aliasing Method** to **FXAA** (`r.AntiAliasingMethod=1`), or keep deformations slow. Rigid (`bRigid`) flexcomps attach as a static surface and do not deform, and raw `<flex>` elements without the macro are not directly supported; author with `<flexcomp>` instead.

## Next steps

- [Articulations](articulations.md) for adding geoms in the Blueprint editor.
- [Debug Visualization](debug.md) for the collision, contact, and segmentation overlays.
