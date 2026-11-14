# Articulations

In URLab, a MuJoCo articulation is an Actor or Blueprint carrying a `UMJModelComponent` alongside a hierarchy of `MuJoCoSpec` components. When compiled, `UMJModelComponent` marshals the component hierarchy directly into a MuJoCo `mjSpec` and compiles `mjModel` + `mjData`. See [The Component Model](../concepts/model.md) for the underlying mechanics.

## The component tree

`UMJModelComponent` owns the `mjSpec*`, `mjModel*`, `mjData*`, `mjVFS*`, `AssetLibrary` (`UMJAssetLibrary`), and `DefaultClasses` (`TArray<FMJDefaultClassEntry>`). The spatial and non-spatial components on the same Actor define the model elements:

```
Actor
├── Model (UMJModelComponent)
├── BaseBody (UMJBodyComponent)
│   ├── Geom_Box (UMJGeomComponent)
│   ├── HingeJoint (UMJJointComponent)
│   └── ChildBody (UMJBodyComponent)
│       └── Site_Tip (UMJSiteComponent)
├── ShoulderActuator (UMJActuatorComponent)
├── TipSensor (UMJSensorComponent)
└── Tendon (UMJTendonComponent)
```

## Building from scratch

1. Create an `AActor` or Blueprint Actor and add a `UMJModelComponent`.
2. **Add bodies and frames.** Add `UMJBodyComponent` (and optional `UMJFrameComponent`) scene components, nesting bodies to form kinematic chains.
3. **Add geoms and sites.** Add `UMJGeomComponent` under a body and configure `FMJGeomSpec` (type, size, `fromto`, density/mass) and `MaterialRef` / `MeshRef` (`FMJElementRef`). Add `UMJSiteComponent` for attachment/sensor sites. See [Geometry & Collision](geometry.md).
4. **Add joints.** Add `UMJJointComponent` under a body and configure its type (`hinge`, `slide`, `ball`, `free`), axis, range, stiffness, and damping in `FMJJointSpec`.
5. **Add actuators, sensors, and tendons.** Add `UMJActuatorComponent` (pick `EMJActuatorShortcut` such as `General`, `Motor`, `Position`, `Velocity`, etc., mapped to `mjs_setTo*`), `UMJSensorComponent`, and `UMJTendonComponent`.
6. **Attach sub-models.** Use `UMJAttachComponent` to attach another model via MuJoCo 4 `mjs_attach`.

## Editor UX: Details customizations & visualizers

URLab registers custom Details panels (`MJSpecDetails`) and viewport component visualizers (`MJSpecVisualizers`) in `URLabEditor` to streamline authoring:

- **Actuator shortcut switcher (`FMJActuatorDetails`):** On `UMJActuatorComponent`, changing `Shortcut` (`EMJActuatorShortcut`) dynamically filters **MuJoCo|Actuator > Active Shortcut Parameters** to show only the fields relevant to that shortcut (`Motor`, `Position`, `Velocity`, `IntVelocity`, `Damper`, `Cylinder`, `Muscle`, `Adhesion`, `PID`, `Orientation`, `DCMotor`), or displays a note when `General` uses `Spec` fields directly.
- **From-To endpoint toggles (`FMJGeomDetails`, `FMJSiteDetails`):** On `UMJGeomComponent` and `UMJSiteComponent`, `bUseFromTo`, `From`, and `To` are grouped under **From-To Endpoints**; `From` and `To` appear only when `bOverride_FromTo` and `bUseFromTo` are enabled.
- **Viewport visualizers (`MJSpecVisualizers`):**
  - `UMJJointComponent` (`FMJJointVisualizer`): draws a directional arrow along `Spec.Axis` plus a type indicator (circle for `Hinge`, back-line for `Slide`, wireframe sphere for `Ball`).
  - `UMJSiteComponent` (`FMJSiteVisualizer`): draws local RGB axes and a wireframe shape (`Box`, `Capsule`, `Cylinder`, or `Sphere`) using `Spec.Size` and `Spec.Rgba`.
  - `UMJFrameComponent` (`FMJFrameVisualizer`): draws local RGB coordinate axes.

## Compiling, stepping, and exporting

- Call `BuildSpecFromActor(OutError)` followed by `CompileModel(OutError)` (or `RecompileModel(OutError)` to preserve `qpos` / `qvel` across edits).
- Call `StepSimulation(Dt)` and `SyncTransformsFromData()` to advance MuJoCo and update `UMJBodyComponent` transforms.
- Call `ExportXML(OutXml, OutError, bSaveCompiled, bSaveCanonical, bSaveInertial)` (or `FMJXmlImporterExporter::ExportActorToXML` in the editor) to serialize the spec to MJCF XML.
