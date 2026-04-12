# Articulation Builder Guide

This guide covers how to build and edit MuJoCo articulations in the Unreal Blueprint editor — both imported from MJCF XML and built from scratch.

---

## Overview

An `AMjArticulation` Blueprint contains a hierarchy of MuJoCo components that mirror the MJCF XML structure. When placed in a level and simulated, the plugin converts this hierarchy into a MuJoCo spec, compiles it, and runs physics.

The component hierarchy is organized into folders:

```
ArticulationRoot
├── worldbody (MjWorldBody)
│   └── body1 (MjBody)
│       ├── Geom_Box (MjBox)
│       ├── HingeJoint (MjHingeJoint)
│       └── body2 (MjBody)
│           └── ...
├── DefinitionsRoot
│   ├── DefaultsRoot
│   │   └── main (MjDefault)
│   │       └── robot (MjDefault)
│   ├── ActuatorsRoot
│   │   └── MotorActuator (MjMotorActuator)
│   ├── SensorsRoot
│   │   └── JointposSensor (MjJointPosSensor)
│   ├── TendonsRoot
│   ├── ContactsRoot
│   ├── EqualitiesRoot
│   └── KeyframesRoot
```

---

## Importing from MJCF XML

1. Drag an `.xml` file into the Content Browser
2. The plugin auto-converts meshes and generates the Blueprint
3. See [MJCF Import Guide](mujoco_import.md) for details

---

## Building from Scratch

### Creating the Blueprint

1. Right-click in Content Browser → Blueprint Class → select `MjArticulation`
2. Open the Blueprint editor

### Adding Bodies

1. In the Components panel, click **Add** → search for `MjBody`
2. Attach it as a child of `worldbody`
3. Set the body's transform (location/rotation)
4. Nest bodies to create kinematic chains (e.g. upper arm → forearm → hand)

### Adding Geoms (Collision/Visual Shapes)

1. Select a body → **Add** → choose a geom type:
   - `MjBox`, `MjSphere`, `MjCylinder` — primitive shapes with built-in visualizers
   - `MjMeshGeom` — for custom static mesh assets (auto-sets type to Mesh)
2. Geoms attach as children of bodies
3. Set size via the UE scale gizmo (primitives) or assign a mesh asset (mesh geoms)
4. **Material Override**: primitives support an `OverrideMaterial` property for visual customization

### Adding Joints

1. Select a body → **Add** → choose a joint type:
   - `MjHingeJoint` — 1-DOF rotation
   - `MjSlideJoint` — 1-DOF translation
   - `MjBallJoint` — 3-DOF spherical
   - `MjFreeJoint` — 6-DOF (root bodies)
2. Joints attach as children of the body they actuate
3. Configure axis, limits, stiffness, damping in the detail panel

### Adding Actuators

1. **Add** → search for actuator type (e.g. `MjMotorActuator`, `MjPositionActuator`)
2. Actuators are **auto-parented** to `ActuatorsRoot` when added
3. Use the **Target** dropdown in the detail panel to select which joint/tendon/site the actuator controls

### Adding Sensors

1. **Add** → search for sensor type (e.g. `MjJointPosSensor`, `MjTouchSensor`)
2. Sensors are **auto-parented** to `SensorsRoot`
3. Use the **Target** dropdown to select what the sensor monitors

### Setting Up Default Classes

Default classes define shared properties (friction, damping, etc.) that multiple components can inherit from.

1. **Add** → `MjDefault` — auto-parented to `DefaultsRoot`
2. The component's **variable name** in the SCS tree becomes the MuJoCo class name (synced at compile time)
3. Add child components under the default to define template values (e.g. add an `MjGeom` child to set default friction)
4. **Nest defaults** to create inheritance: drag one default under another in the SCS tree. The parent-child relationship is inferred from the hierarchy.
5. On bodies, use the **Child Class** dropdown to assign a default class to all children
6. On individual geoms/joints, use the **Default Class** dropdown to reference a specific default

---

## Component Reference Dropdowns

Most cross-references between components use dropdown pickers instead of string names:

| Component | Dropdown | Shows |
|-----------|----------|-------|
| Actuator | Target | Joints, Tendons, Sites, or Bodies (filtered by Transmission Type) |
| Sensor | Target | Bodies, Joints, Geoms, Sites, etc. (filtered by Object Type) |
| Sensor | Reference | Bodies, Geoms (for relative sensors) |
| Contact Pair | Geom 1/2 | All geom components |
| Contact Exclude | Body 1/2 | All body components |
| Equality | Object 1/2 | Bodies, Joints, or Tendons (filtered by Equality Type) |
| Body | Child Class | All default class components |
| Geom/Joint/etc. | Default Class | All default class components |

The dropdowns read and write the underlying string properties used by MuJoCo spec construction.

---

## Auto-Parenting

When you add certain component types, they are automatically moved to the correct organizational folder:

| Component Type | Auto-Parented To |
|---------------|------------------|
| Sensor | SensorsRoot |
| Actuator | ActuatorsRoot |
| Default | DefaultsRoot |
| Tendon | TendonsRoot |
| Contact Pair/Exclude | ContactsRoot |
| Equality | EqualitiesRoot |

Components under `DefaultsRoot` (default templates) are excluded from auto-parenting — they stay where you put them.

---

## Compiling and Validating

Press **Compile** in the Blueprint editor to:
- Sync default class names from the SCS variable names
- Sync parent-child default relationships from the SCS hierarchy
- Run `ValidateSpec` which creates a temporary MuJoCo spec and checks for errors

Compilation errors appear in the Output Log (e.g. missing joint references, invalid ranges).

---

## MuJoCo Outliner

Open **Window → MuJoCo Outliner** for a filtered view of the articulation:

- **BP selector** — choose which open articulation to inspect
- **Type filters** — toggle Bodies, Geoms, Joints, Sensors, Actuators, etc.
- **Search** — filter by component name
- **Click to select** — clicking an entry selects it in the Blueprint editor's component tree
- **Summary bar** — shows component counts

---

## Custom Icons

Each MuJoCo component type has a distinct icon in the component tree:

- **Orange** — Bodies (bone shape)
- **Green** — Geoms (box, sphere, cylinder, capsule, mesh, triangle wireframe)
- **Blue** — Joints (angle arc, arrow, starburst)
- **Purple** — Sensors (eye shape)
- **Red** — Actuators (gear)
- **Grey** — Defaults (document)
- **Cyan** — Sites (pin marker)
- **Teal** — Tendons (taut line)

---

## Tips

- **Name your components** — the SCS variable name is used as the MuJoCo element name. Clear names make debugging easier.
- **Use defaults for shared properties** — instead of setting friction on every geom, create a default class and reference it.
- **Check the Output Log** after compile for validation warnings.
- **Defaults are hierarchical** — nest defaults in the SCS tree to create inheritance chains (child inherits from parent).
