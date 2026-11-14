# Actuators & Control

In `MuJoCoSpec`, actuators are authored with `UMJActuatorComponent` and driven either in C++ via `UMJModelComponent` or externally from Python via `SimulationClient`.

## `UMJActuatorComponent`

`UMJActuatorComponent` (`Source/URLab/Public/MuJoCoSpec/MJActorComponents.h`) represents an MJCF `<actuator>` on a `UMJModelComponent` Actor:

- **`Shortcut` (`EMJActuatorShortcut`):** Selects a MuJoCo 4 actuator shortcut (`General`, `Motor`, `Position`, `Velocity`, `IntVelocity`, `Damper`, `Cylinder`, `Muscle`, `Adhesion`, `PID`, `Orientation`, `DCMotor`), mapped to `mjs_setTo*` during `BuildSpecFromActor`.
- **`ShortcutParams` (`FMJActuatorShortcutParams`):** Holds shortcut-specific parameters (`Kp`, `Kv`, `Ki`, `DampRatio`, `TimeConst`, `Gain`, `Bias`, `Area`, `Diameter`, `InheritRange`, etc.). In the editor, `FMJActuatorDetails` automatically filters **Active Shortcut Parameters** to show only the fields used by the selected `Shortcut`.
- **`TargetRef` (`FMJElementRef`):** References the driven joint, tendon, or site.
- **`Ctrl` (`double`):** Initial/authored control value written to `mjData->ctrl`.

## Driving a simulation from Python

Runtime control authority belongs to the world-owned simulation service (`-URLabWorkflow=simulation`), not to individual components:

1. Connect with `urlab_client.Connection` and create a `SimulationClient`.
2. Explicitly claim control authority (`claim_control`).
3. Choose stepped or freerun clock (`-URLabClock=stepped|freerun`) and send control vectors / step commands.

See [Python & External Control](../python/index.md) and [Simulation, render server and viewer](../concepts/render_roles.md) for the service contract.
