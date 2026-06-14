# Controllers

A controller decides how a control target becomes joint torques. Attach one component to an articulation to intercept the control pipeline at physics rate, or let MuJoCo handle control directly when no controller is present.

## How control flows

A control target arrives on each `UMjActuator` from ZMQ or Blueprint. When the articulation applies controls each physics step:

- If the articulation has a controller component, the controller's compute step runs and writes `d->ctrl` (or torques) using its own control law.
- If there is no controller, the raw target is written straight to `d->ctrl`. This is the right behaviour for position actuators, where MuJoCo runs the PD loop internally.

## Built-in controllers

- **Passthrough** (`UMjPassthroughController`): a direct write, the same as no controller but explicit.
- **PD** (`UMjPDController`): turns a position target into torque at physics rate, for use with `<motor>` actuators.

### PD controller

The PD controller computes `torque = Kp * (target - qpos) - Kv * qvel` each physics step, clamps the target to joint range, and applies torque limits. Configure it with per-actuator gain arrays and fallbacks:

| Property | Description |
|---|---|
| `Kp` | Proportional gains, one per actuator |
| `Kv` | Derivative gains |
| `TorqueLimits` | Symmetric torque clamps |
| `DefaultKp` / `DefaultKv` / `DefaultTorqueLimit` | Fallback for any unset entry |

Gains can also be pushed live from Python over ZMQ as name-keyed JSON, so you can tune a policy without recompiling. See [Python Policies](../python/policies.md).

## Sequencing keyframe poses

`UMjKeyframeController` plays through a list of named poses with ease-in-out blending. Unlike the PD and passthrough controllers, it writes actuator targets at game-tick rate (via `SetActuatorControl`) rather than intercepting the physics-rate pipeline, so it layers on top of whatever controller drives the torques.

Add it to an articulation. Each pose has a name, a list of actuator targets ordered to match the articulation's actuators, a hold time, and a blend time. The component ships with presets for common robots; call `GetPresetNames()` to list them and `LoadPreset(Name)` to populate the pose list, or pick a preset in the Details panel. Use `Play()`, `Stop()`, and `GoToKeyframe(Index)` to drive playback, with `bAutoPlay`, `StartDelay`, and `bLoop` controlling automatic behaviour.

This component sequences poses through actuator targets. For snapping directly to MJCF keyframes (qpos/qvel/ctrl), use the articulation's keyframe API in [Articulations](articulations.md).

## Writing a custom controller

Derive from `UMjArticulationController` and override the compute step. The base class populates a `Bindings` array after the model compiles, mapping each actuator to its joint's qpos and qvel addresses:

```cpp
UCLASS(ClassGroup=(Custom), meta=(BlueprintSpawnableComponent))
class UMyController : public UMjArticulationController
{
    GENERATED_BODY()
public:
    UPROPERTY(EditAnywhere)
    float MyGain = 50.0f;

    virtual void ComputeAndApply(mjModel* m, mjData* d, uint8 Source) override
    {
        for (int32 i = 0; i < Bindings.Num(); ++i)
        {
            const FActuatorBinding& B = Bindings[i];
            float Target = B.Component->ResolveDesiredControl(Source);
            float Pos = (float)d->qpos[B.QposAddr];
            float Vel = (float)d->qvel[B.QvelAddr];
            d->ctrl[B.ActuatorMjID] = (mjtNum)(MyGain * (Target - Pos));
        }
    }
};
```

Binding runs once after the model compiles. It resolves each actuator's joint transmission, stores the qpos/qvel addresses, and sorts entries by actuator ID so the index order matches how Python discovers actuators. Free and ball joints are skipped.

!!! warning
    The compute step runs on the async physics thread. Read control targets through `ResolveDesiredControl()` (atomic). Gain arrays tolerate benign torn reads during updates, since individual float writes are atomic on x86 and ARM. Do not touch non-thread-safe engine state from inside it.

To use a controller, add it to the articulation Blueprint, configure its properties (or push them from Python), and it activates automatically once enabled and bound.

## Choosing the control source

The compute step receives a `Source` argument so it can resolve the right target. The global control source lives on the physics engine:

```cpp
Manager->PhysicsEngine->SetControlSource(EControlSource::ZMQ); // or ::UI
```

Each articulation can override the global setting with its own `ControlSource` (`0` = ZMQ, `1` = UI), so some robots run from the dashboard while others take an external policy in the same scene. See [Articulations](articulations.md) and the [Simulate Dashboard](dashboard.md) for the in-editor toggle.
