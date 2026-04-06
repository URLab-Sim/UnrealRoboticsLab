# Geometry & Collision

How to define collision shapes — from primitives to decomposed meshes — and control which bodies collide.

---

## Geometry Types

| Type | Component | Defined by |
|------|-----------|-----------|
| Box | `UMjBox` | Half-extents (via scale XYZ) |
| Sphere | `UMjSphere` | Radius (via scale X) |
| Cylinder | `UMjCylinder` | Radius (scale X) + half-height (scale Z) |
| Capsule | `UMjGeom` (type=Capsule) | Radius + half-height |
| Mesh | `UMjGeom` (type=Mesh) | Static Mesh asset |
| Heightfield | `AMjHeightfieldActor` | Unreal Landscape data |

Use the standard Unreal transform gizmo to size shapes — the plugin reads component scale and maps it to MuJoCo size parameters.

---

## Primitives

Fast, exact analytical collision. Add `UMjBox`, `UMjSphere`, or `UMjCylinder` as children of a `UMjBody`. Stack multiple on one body for compound shapes.

## Mesh Geoms

For convex meshes, set `Type = Mesh` and assign a Static Mesh. MuJoCo computes the convex hull internally.

For **concave** meshes, use CoACD decomposition — click "Decompose Mesh" on the geom, or set `ComplexMeshRequired` on a Quick Convert component.

---

## Quick Convert

`UMjQuickConvertComponent` on any Static Mesh actor — one-click physics body.

| Property | Effect |
|----------|--------|
| `Static` | Fixed in place (no free joint) |
| `ComplexMeshRequired` | CoACD decomposition for concave meshes |
| `CoACDThreshold` | Decomposition detail (0.01–1.0, lower = more convex pieces) |
| `bDrivenByUnreal` | Unreal drives position, MuJoCo does not simulate |

## Heightfield Terrain

`AMjHeightfieldActor` converts Unreal Landscape geometry into a MuJoCo heightfield for terrain contact. Enable `bTraceComplex` for accurate mesh-level sampling of the landscape. Use `TraceWhitelist` to filter which actors are sampled (e.g., only terrain, not foliage).

---

## Geom Properties

| Property | Description |
|----------|-------------|
| `Friction` | Sliding, torsional, rolling coefficients |
| `Density` | Mass inferred from density × volume |
| `Margin` | Contact detection margin |
| `Solref` / `Solimp` | Solver stiffness/damping/impedance |

---

## Contact Filtering

**Groups** (0–5): integer tags on geoms used for organizing visibility in MuJoCo's native viewer. In URLab, group 2 is conventionally used for visual-only meshes (contype=0, conaffinity=0) and group 3 for collision-only meshes. Contact generation is controlled by `contype` and `conaffinity` bitmasks, not groups.

**Contact Pairs** (`UMjContactPair`): explicitly enable contact between two bodies.

**Contact Exclusions** (`UMjContactExclude`): disable contact between two bodies.

**Bitmask** (`Contype` / `Conaffinity`): scalable filtering for large scenes. Two geoms collide only if their type/affinity masks overlap.

---

## Debug Visualization

Collision debug viz draws geoms that belong to **group 3** OR have **contype != 0 && conaffinity != 0**. This ensures foot contact spheres (which may not be in group 3 but do participate in contacts) are visible in the debug overlay.

Press **2** to toggle visual mesh rendering. When visual meshes are hidden, only collision geometry is drawn — useful for inspecting contact shapes without visual clutter.

---

## Inertia Override

MuJoCo infers mass from geom density and shape. For explicit control, add `UMjInertial` to the body with a specific mass, center of mass, and inertia matrix.
