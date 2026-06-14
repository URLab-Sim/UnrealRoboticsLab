# Controllers and Gains

Remote stepping respects URLab's existing controller framework. Each articulation either carries a `UMjArticulationController` (PD today, others later) or writes raw actuator signals. This page covers how the client picks between the two, how decimation works via `n_steps`, and how to patch gains from Python.

See also the [Controller Framework guide](../guides/controller_framework.md) for the UE-side C++ picture.

## Control modes: `ue_controller` vs. `raw`

Selectable per articulation per step.

| Mode | Behaviour | Default when... |
|---|---|---|
| `ue_controller` | Step request writes ctrl into the actuator's `NetworkValue` atomic. UE's controller runs every physics sub-step of the `n_steps` window against that fixed setpoint. Policy emits setpoints (position targets for PD, velocity targets for a velocity controller, ...); UE owns signal generation. | A `UMjArticulationController` is attached |
| `raw` | Step request writes directly to `d->ctrl`. UE's controller is suppressed for this articulation for this step. Policy emits raw actuator signals. | No controller is attached |

Mode is per-articulation, so a scene with a PID-controlled manipulator plus a raw-torque quadruped just works. The default comes from what is wired in UE; most users never touch it.

Override from Python before `step()`:

```python
art = client.articulations["vx300s"]
art.control_mode = "raw"   # or "ue_controller"
```

In puppet mode the control mode is ignored (UE controllers do not run). A `control_mode` in the step request triggers a one-line debug warning and is otherwise dropped.

## Decimation via `n_steps`

In `ue_controller` mode, `n_steps` picks how many physics sub-steps run per `step()` call. The controller's inner loop sees fresh `d->qpos` / `d->qvel` every sub-step against the same setpoint. Example cadence:

```python
# 500Hz physics, 100Hz policy: five physics sub-steps per policy step.
client.step(n_steps=5)
```

This mirrors how mjlab runs its inner loop. The controller integrates cleanly because the sub-steps are contiguous in `d->time`.

In `raw` mode `n_steps` still advances the sim, but `d->ctrl` is held constant across sub-steps (the client wrote it once) and no inner loop runs. The effect is identical to calling `mj_step` with the same ctrl five times.

## Introspecting the attached controller

The handshake ships a kind-tagged config block per articulation when a controller is attached. The Python client wraps it as `art.controller`.

```python
art = client.articulations["vx300s"]

art.controller.kind          # "pd" (matches UE class short name)
art.controller.params        # dict of current server-side values
art.controller.schema        # dict describing type / dtype / bounds per param
```

For PD the params look like:

```python
{
    "kp":           {"waist": 300.0, "shoulder": 280.0, "...": "..."},
    "kv":           {"waist": 20.0,  "shoulder": 18.0,  "...": "..."},
    "torque_limit": {"waist": 35.0,  "shoulder": 45.0,  "...": "..."},
    "default_kp": 100.0,
    "default_kv": 5.0,
    "default_torque_limit": 200.0,
}
```

Call `art.controller.refresh()` to re-pull values from UE (for example, after another client or the editor patched them).

## PD gains: typed setters

The `URLabPDController` subclass exposes typed setters so the common case does not force a dict-of-dicts round-trip:

```python
art.controller.set_gains(
    kp={"waist": 300.0, "shoulder": 280.0},
    kv={"waist": 20.0,  "shoulder": 18.0},
    torque_limit={"waist": 35.0, "shoulder": 45.0},
)

art.controller.set_defaults(kp=100, kv=5, torque_limit=200)

art.controller.kp   # live dict view, tracks server state
art.controller.kv
```

### Partial patch semantics

Any joint not mentioned keeps its current value. It is **not** reset to default. The only exception is `set_defaults`, which rebinds every joint to the new default.

This matches the existing `{prefix}/set_gains` PUB/SUB topic: same model for the editor, the live streaming path, and the step server. Under the hood all three routes call `UMjArticulationController::ApplyConfig`.

Setters are safe in any mode. They send the RPC regardless of `live` / `direct` / `puppet`. Under `raw` the controller params are still tracked server-side (for when the user flips back to `ue_controller`) but have no immediate effect on physics.

## Generic configure for custom controllers

If you ship a controller kind that does not have a typed Python subclass, the generic path works:

```python
art.controller.configure(
    my_gain=12.3,
    my_clamp={"joint_a": 5.0},
)
```

`configure` validates against `art.controller.schema`, sends the RPC, and returns the full server-acknowledged params dict. Rejected payloads come back as `error(code="controller_schema_violation")`.

## Adding a new controller kind (advanced)

On the UE side:

1. Subclass `UMjArticulationController`, implement `ComputeAndApply` as normal.
2. Override `GetKindName()` so the handshake can report the kind string.
3. Override `GetConfigSchema(TSharedPtr<FJsonObject>&)` to describe the params.
4. Override `GetCurrentConfig(...)` and `ApplyConfig(...)` for read/write.

On the Python side, the generic `URLabController` wraps the new kind automatically using the shipped schema. Ship a typed subclass (like `URLabPDController`) only if you want named-argument ergonomics.

Three callers converge on the same `ApplyConfig` path: the direct `SetGains` API, the `{prefix}/set_gains` JSON topic, and the `configure_controller` RPC. One schema, one validator, one apply path.
