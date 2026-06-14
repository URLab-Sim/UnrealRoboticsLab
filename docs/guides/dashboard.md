# Simulate Dashboard

The Simulate dashboard is an in-editor widget that mirrors MuJoCo's `simulate` UI. Drive actuators, read sensors, watch camera feeds, jump to keyframes, and control playback without leaving Play-in-Editor.

![The Simulate dashboard with actuator sliders, sensor readouts, and a camera feed panel](../images/placeholder.svg)

## Showing the dashboard

The dashboard appears automatically when you hit Play, as long as:

1. An `MjManager` actor is present in the level (one per level).
2. `bAutoCreateSimulateWidget` is enabled on the Manager.
3. **Show Plugin Content** is enabled in the Content Browser settings (gear icon), so the bundled widget assets are visible to the engine.

!!! note
    If the dashboard does not appear, check those three conditions first. The widget assets ship as plugin content, so without **Show Plugin Content** the engine cannot find them.

## What it does

**Top bar.** Simulation time and a real-time factor, with Play/Pause and Reset buttons for the physics thread.

**Articulation selector.** Pick which articulation the controls apply to. The actuator sliders, sensor readouts, keyframe dropdown, and camera feeds all follow this selection.

**Actuator sliders.** One slider per actuator on the selected articulation, ranged to the actuator's limits. Move a slider to set the control target directly.

!!! tip
    Sliders only take effect when the control source is set to UI. If it is set to ZMQ, the dashboard is ignored in favour of external commands. Set the source on the Manager (through the physics engine) or per-articulation. See [Controllers](controllers.md).

**Sensor readouts.** Live values for the selected articulation's sensors, updated each tick. See [Sensors & Cameras](sensors_cameras.md) for the categories and what each returns.

**Camera feeds.** A panel showing every `UMjCamera` on the selected articulation, each rendering its configured capture mode live (RGB, depth, or segmentation) with no extra setup.

**Keyframe dropdown.** Lists the keyframes on the selected articulation, with **Reset to Keyframe** and a **Hold / Stop Hold** toggle. Reset snaps the pose; Hold maintains it continuously. See [Articulations](articulations.md).

**Snapshots.** Capture the full simulation state and restore it later, for A/B testing or hand-authoring poses. Combine with paused mouse perturbation to dial in a pose by hand, then snapshot it, as described in [Interaction](interaction.md).

**Recording and replay.** Start and stop recording, load a saved session, and pick a session to replay. See [Recording & Replay](recording.md).

**Possession and locomotion.** When the selected articulation has a twist controller, a **Possess** button and locomotion speed sliders appear so you can drive it with the keyboard. See [Interaction](interaction.md).

## Next steps

- [Interaction](interaction.md) for possession, twist control, and mouse perturbation.
- [Debug Visualization](debug.md) for the PIE overlays and their hotkeys.
- [Recording & Replay](recording.md) for capturing and playing back episodes.
