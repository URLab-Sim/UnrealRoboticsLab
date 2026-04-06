# Possession & Twist Control

URLab articulations are Unreal Pawns — they can be possessed by a PlayerController to enable direct input control. The twist controller captures WASD/gamepad input and broadcasts velocity commands over ZMQ.

## Possessing an Articulation

**From the MjSimulate widget:** Click the **Possess** button next to the articulation selector. The button toggles to **Release** while possessed.

**From Blueprint/C++:**
```cpp
APlayerController* PC = GetWorld()->GetFirstPlayerController();
PC->Possess(MyArticulation);
// Later:
PC->UnPossess();
```

When possessed:
- Enhanced Input mapping context (`IMC_TwistControl`) is added
- A spring arm + camera attaches to the root MjBody for follow-cam
- WASD/QE input feeds the twist controller

When released:
- Mapping context removed, twist state zeroed
- Camera components cleaned up
- Original pawn re-possessed

## UMjTwistController

Auto-created on every AMjArticulation at BeginPlay. Captures input and stores thread-safe twist state.

**Properties (editable in Details panel or via locomotion sliders):**

| Property | Default | Description |
|----------|---------|-------------|
| `MaxVx` | 0.8 m/s | Max forward speed |
| `MaxVy` | 0.5 m/s | Max strafe speed |
| `MaxYawRate` | 1.57 rad/s | Max turn rate |

**Input mapping:**

| Key | Action |
|-----|--------|
| W/S | Forward/backward |
| A/D | Strafe left/right |
| Q/E | Turn left/right |
| 1-0 | Action keys (10-slot bitmask) |

**ZMQ broadcast:** The sensor broadcaster publishes twist state every physics step:

| Topic | Payload |
|-------|---------|
| `<prefix>/twist` | `3 x float32`: vx (m/s), vy (m/s), yaw_rate (rad/s) |
| `<prefix>/actions` | `int32` bitmask of pressed action keys |

## Follow Camera

On possession, a spring arm + camera component attaches to the root MjBody:

- **Camera lag**: speed 8, rotation lag enabled for smooth tracking
- **Arm length**: 300 units behind, 100 units above
- Tagged `PossessCamera` for cleanup on release

The camera follows the actual physics body position, not the static actor root.

## Connecting to Policies

The Python policy bridge reads twist commands via the `UnrealTwistCtrl` controller class:

```python
# In policy_gui.py — twist controller masquerades as JoystickCtrl
# so the Unitree policy's _get_commands() finds the axes data
ctrl_list = [UnrealTwistCtrlCfg()]
```

Twist values map to policy commands:
- `vx` → forward velocity command
- `vy` → lateral velocity command
- `yaw_rate` → turn command

The policy interprets these as walking direction/speed.

## Locomotion Sliders

When an articulation with a TwistController is selected in the MjSimulate widget, a **LOCOMOTION** section appears with three sliders:

- **Max Forward Speed** (0.0 – 2.0 m/s)
- **Max Strafe Speed** (0.0 – 1.0 m/s)
- **Max Turn Rate** (0.0 – 3.14 rad/s)

These control how much velocity a full keypress produces.

## Global Simulation Hotkeys

See [Hotkeys](blueprint_reference.md#hotkeys) for all keyboard shortcuts.

## MjKeyframeCameraActor

For scripted camera work (e.g., filming demos), use `MjKeyframeCameraActor` instead of the possession follow-cam. It plays back predefined camera keyframes with interpolation, independent of any possessed pawn.

## Input Assets

Located in `Content/Input/`:
- `IA_TwistMove` — Axis2D input action (WASD)
- `IA_TwistTurn` — Axis1D input action (Q/E)
- `IMC_TwistControl` — Input mapping context binding both actions

Auto-loaded from plugin content when the TwistController is created.
