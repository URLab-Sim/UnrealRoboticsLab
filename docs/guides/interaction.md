# Interaction

Manipulate bodies with the mouse during Play-in-Editor, and possess an articulation to drive it directly with the keyboard.

## Mouse perturbation

Drag bodies during PIE with gestures that match MuJoCo's `simulate` viewer. Under the hood URLab calls the same perturbation primitives, so the spring feel, force falloff, and paused-teleport behaviour are identical to upstream. Perturbation runs on `UMjPerturbation`, auto-created on the Manager.

![Dragging a body with the perturbation spring gizmo during PIE](../images/placeholder.svg)

### Gestures

| Input | Action |
|---|---|
| Double-click LMB on a body | Select it. No force applied. The selection persists until you double-click elsewhere or on empty space. |
| Ctrl + RMB drag | Translate the selected body along the cursor plane at the selection depth. |
| Ctrl + LMB drag | Rotate the selected body about the camera-aligned axis. |
| Release the button | Stop the drag. Selection persists; forces clear on the next physics step. |

Camera look and move input is disabled while a drag is active so cursor motion drives the body, not the view.

### Running versus paused

- **Running**: the drag applies a virtual spring force, so the body integrates under physics. It can collide, swing, and settle. Releasing clears the force on the next step.
- **Paused** (press `P`): the drag teleports the body to the reference pose and propagates it through the kinematics. Hold Ctrl and drag to pose the body statically; releasing keeps the pose frozen.

While dragging, gizmos overlay the body: a red sphere at the selection point, a green arrow during translate (selection point to target, length is the position error), and a yellow arrow during rotate (the linear motion the rotation spring is producing).

!!! tip
    Combine paused perturbation with snapshots in the [Simulate Dashboard](dashboard.md) to author keyframes by hand: pause, drag the body into position, capture a snapshot, then resume. The snapshot records the current qpos and ctrl, so the pose you dialled in replays exactly.

## Possession and keyboard control

URLab articulations are Unreal Pawns, so a PlayerController can possess one for direct input control. The twist controller (auto-created on every articulation) captures WASD/gamepad input and broadcasts velocity commands over ZMQ.

### Possessing

From the [Simulate Dashboard](dashboard.md), click **Possess** next to the articulation selector; the button toggles to **Release** while possessed. From Blueprint or C++:

```cpp
APlayerController* PC = GetWorld()->GetFirstPlayerController();
PC->Possess(MyArticulation);
// Later:
PC->UnPossess();
```

On possession, URLab adds the twist input mapping context and attaches a spring-arm follow camera to the root body, which tracks the physics body rather than the static actor root. Releasing removes the mapping, zeroes the twist state, cleans up the camera, and re-possesses your original pawn.

### Twist input

| Key | Action |
|-----|--------|
| W/S | Forward/backward |
| A/D | Strafe left/right |
| Q/E | Turn left/right |
| 1 to 0 | Action keys (a 10-slot bitmask) |

The twist controller publishes its state every physics step: a `<prefix>/twist` topic carrying vx, vy, and yaw rate, and a `<prefix>/actions` topic carrying the pressed-action bitmask. A Python policy can subscribe to these and interpret them as walking direction and speed. See [Python Policies](../python/policies.md).

Tune how much velocity a full keypress produces with the max forward speed, strafe speed, and turn rate. These are editable in the Details panel or, when a twist-controlled articulation is selected, from a locomotion section of sliders in the dashboard.

!!! note
    For scripted camera moves (filming demos), use a keyframe camera actor that interpolates through predefined waypoints, independent of any possessed pawn, instead of the possession follow-cam.

## Next steps

- [Simulate Dashboard](dashboard.md) for the Possess button, locomotion sliders, and snapshots.
- [Debug Visualization](debug.md) for the hotkeys and overlays, including wireframes useful for diagnosing rest poses.
- [Recording & Replay](recording.md) to capture an interaction session.
