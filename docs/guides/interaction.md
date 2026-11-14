# Interaction

Apply external forces to a running simulation from Python, and possess an articulation to drive it directly with the keyboard.

## External perturbations

Perturbations are typed, granted wrenches. The simulation owner issues a short-lived grant (at most 30 s) bound to its control authority, its pose-stream source and one viewer; the grant holder submits wrenches through that viewer (`viewer_interaction_submit`) or straight to the producer (`interaction_submit`). Revoking the grant, letting it expire, or any change of source, control or clock epoch stops every wrench issued under it before the next step. Withdrawing the `accept_input` permission revokes all grants.

| Producer | Issue a grant | Owner-side application |
|---|---|---|
| UE simulation (`-URLabWorkflow=simulation`) | `SimulationClient.issue_interaction_grant(handle, source, viewer_boot_id, ttl_s)` | built in |
| Python `PoseStreamPublisher` | `publisher.issue_interaction_grant(viewer_boot_id, ttl_s)` | `MujocoInteractionHandler(model, data)` plus `before_authoritative_step(data.time)` / `after_authoritative_step(...)` around `mj_step` (`abort_authoritative_step()` if the step fails) |

### Frames and units

Wrenches are in MuJoCo coordinates: right-handed, Z up, metres, newtons, newton-metres. They are never in Unreal units; anything that starts from UE coordinates must convert centimetres to metres and flip Y. By default force and torque are world-frame and act at the body centre of mass. With `body_frame=True` they are expressed in the body frame and re-rotated every step. With `point_m` the force acts at that point (world coordinates, or body coordinates relative to the body origin when `body_frame=True`) and the owner adds the matching torque about the centre of mass.

### Impulses, holds and drags

```python
from urlab_client import InteractionDrag, InteractionWrench

viewer.submit_interaction(owner, revision, grant, InteractionWrench(body, (0, 0, 50), (0, 0, 0)))  # one step
viewer.submit_interaction(owner, revision, grant, InteractionWrench.hold(body, (5, 0, 0), duration_s=0.5))

drag = InteractionDrag(functools.partial(viewer.submit_interaction, owner, revision, grant), body, lease_s=0.2)
drag.update((3, 0, 0), point_m=(0.1, 0, 0))   # call faster than lease_s
drag.release()
```

- A plain wrench applies to exactly one authoritative step, an impulse of force times the timestep.
- `duration_s` holds the wrench on every step until the simulation time passes the first step plus `duration_s`, so the impulse is force times duration regardless of how often the client submits.
- A wrench on a `channel` replaces the previous hold on the same grant and channel. `InteractionDrag` uses this to stream a drag; if updates stop, the force lapses after `lease_s` of simulation time.

The reply arrives after the first step that applied the wrench, with that step's `source_time`.

### Retries

Every `InteractionWrench` carries an `interaction_id`. Resubmitting the same value never applies it twice, so after `outcome_unknown` (the viewer could not confirm the producer's result in time) resubmit the same wrench object. Use `wrench.renewed()` for a deliberately new interaction.

The packaged viewer has no mouse pick or drag of its own; drive perturbations from Python.

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

The twist controller's state goes out with the rest of the articulation's state, in the msgpack snapshot published every physics step: a `twist` field carrying linear and angular velocity, and an `actions` field carrying the pressed-action bitmask. A Python policy reads them from the snapshot and interprets them as walking direction and speed. See [Python Policies](../python/policies.md).

Tune how much velocity a full keypress produces with the max forward speed, strafe speed, and turn rate. These are editable in the Details panel or, when a twist-controlled articulation is selected, from a locomotion section of sliders in the dashboard.

!!! note
    For scripted camera moves (filming demos), use a keyframe camera actor that interpolates through predefined waypoints, independent of any possessed pawn, instead of the possession follow-cam.

## Next steps

- [Simulate Dashboard](dashboard.md) for the Possess button, locomotion sliders, and snapshots.
- [Debug Visualization](debug.md) for the hotkeys and overlays, including wireframes useful for diagnosing rest poses.
- [Recording & Replay](recording.md): retired; kept as historical context.
