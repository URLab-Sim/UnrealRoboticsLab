# Controller Framework

The controller framework lets you define how position targets become joint torques. Every articulation can have one controller component that intercepts the control pipeline at physics rate.

## Control Flow

```
ZMQ / Blueprint → SetNetworkControl(target) on UMjActuator
                         │
         AMjArticulation::ApplyControls()
                         │
              ┌──────────┴──────────┐
              │ Controller found?   │ No controller
              │                     │
     Controller.ComputeAndApply()  d->ctrl[id] = target
              │                     (direct write)
     Custom control law runs
     at every physics step
```

If no controller component exists, `ApplyControls` writes the raw `NetworkValue` to `d->ctrl` — suitable for position actuators where MuJoCo handles PD internally.

## Built-in Controllers

| Controller | Use Case |
|-----------|----------|
| **UMjPassthroughController** | Direct write (same as no controller, but explicit) |
| **UMjPDController** | Position target → PD torque at physics rate. For motor actuators. |

## UMjPDController

Computes `torque = Kp * (target - qpos) - Kv * qvel` every physics step. Targets are clamped to joint range. Use with `<motor>` actuators in the MJCF.

**Properties:**

| Property | Type | Description |
|----------|------|-------------|
| `Kp` | `TArray<float>` | Proportional gains, one per actuator |
| `Kv` | `TArray<float>` | Derivative gains |
| `TorqueLimits` | `TArray<float>` | Symmetric torque clamps |
| `DefaultKp/Kv/TorqueLimit` | `float` | Fallback for unset entries |

Gains can be pushed from Python via ZMQ `set_gains` message (name-keyed JSON).

## UMjKeyframeController

A preset-based pose sequencer. Unlike `UMjPDController` and `UMjPassthroughController`, this is a `UActorComponent` (not derived from `UMjArticulationController`) — it writes actuator targets via `SetActuatorControl` at game-tick rate rather than intercepting the physics-rate control pipeline.

Add it to an `AMjArticulation` actor. It auto-resolves the owner articulation on `BeginPlay`.

**FMjKeyframePose struct:**

| Field | Type | Description |
|-------|------|-------------|
| `Name` | `FString` | Display name (e.g., "Reach") |
| `Targets` | `TArray<float>` | Actuator targets, ordered to match the articulation's actuator list |
| `HoldTime` | `float` | Seconds to hold this pose before advancing |
| `BlendTime` | `float` | Seconds to interpolate from the previous pose (ease-in-out) |

**Preset system:** Call `GetPresetNames()` for available presets, then `LoadPreset(Name)` to populate the `Keyframes` array. In the Details panel, select from the `Preset` dropdown and tick `bLoadPreset` to apply.

**Built-in presets:** Franka (Reach-Grasp-Lift, Wave), UR5e (Pick-Place, Sweep), Spot (FR Knee Cycle, Wave, Sit), ANYmal (Sit, Weight Shift), Go2 (Stand-Crouch, Stretch).

**Properties:**

| Property | Default | Description |
|----------|---------|-------------|
| `bAutoPlay` | `true` | Start playing on BeginPlay |
| `StartDelay` | `0.5s` | Delay before first keyframe |
| `bLoop` | `true` | Loop back to first keyframe after last |
| `Preset` | empty | Selected preset name |

**Actions:** `Play()`, `Stop()`, `GoToKeyframe(Index)`, `LoadPreset(Name)`.

## Per-Articulation Control Source

The manager has a global `EControlSource` (UI or ZMQ). Each `AMjArticulation` can override this with its own `ControlSource` field (`uint8`: 0 = ZMQ, 1 = UI). This allows mixed control — some robots driven by the dashboard, others by an external policy — in the same scene.

## Writing a Custom Controller

Derive from `UMjArticulationController` and override `ComputeAndApply`:

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

            float Torque = MyGain * (Target - Pos);
            d->ctrl[B.ActuatorMjID] = (mjtNum)Torque;
        }
    }
};
```

The `Bindings` array is populated automatically by `Bind()` — each entry maps an actuator to its joint's qpos/qvel addresses.

## Binding

`Bind()` runs once after the MuJoCo model compiles. It resolves each actuator's joint transmission via `m->actuator_trnid` and stores the qpos/qvel addresses. Free and ball joints are skipped.

Bindings are sorted by actuator MjID so the array index matches the order Python discovers actuators.

## Thread Safety

`ComputeAndApply` runs on the async physics thread. Read actuator values via `ResolveDesiredControl()` (atomic). Gain arrays accept benign torn reads during updates — individual float writes are atomic on x86/ARM.

## Adding to an Articulation

1. Open your AMjArticulation Blueprint
2. **Add Component** → search for your controller (e.g., `MjPDController`)
3. Configure properties in the Details panel (or push via ZMQ)
4. The controller activates automatically when `bEnabled = true` and `IsBound()`
