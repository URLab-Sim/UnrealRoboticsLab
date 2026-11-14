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
