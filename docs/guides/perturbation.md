# Interactive Perturbation

Mouse-driven manipulation of bodies during PIE. Gesture mapping matches MuJoCo's `simulate` viewer, and under the hood the plugin calls the same `mjv_applyPerturbForce` / `mjv_applyPerturbPose` primitives — so the spring feel, force falloff, and paused-teleport semantics are identical to upstream.

Driven by `UMjPerturbation` (auto-created on `AAMjManager`). `UMjInputHandler` owns the gesture detection.

---

## Gestures

| Input | Action |
|---|---|
| **Double-click LMB** on a body | Select. No force applied. Selection persists until you double-click elsewhere or on empty space. |
| **Ctrl + RMB drag** | Translate the selected body along the cursor plane at the selection depth. |
| **Ctrl + LMB drag** | Rotate the selected body about the camera-aligned axis. |
| Release the mouse button | Stop drag. Selection persists, forces clear on the next physics step. |

Camera look/move input is disabled while a drag is active so the cursor motion drives the body rather than the view.

---

## Running vs paused

| State | Behaviour |
|---|---|
| **Running** | `mjv_applyPerturbForce` writes a virtual spring force into `xfrc_applied`. The body integrates under physics — it can collide, swing, settle. Release and the force is cleared on the next pre-step. |
| **Paused** (`P`) | `mjv_applyPerturbPose` teleports the body to the reference pose and `mj_forward` propagates `qpos → xpos`. Hold Ctrl and drag to pose the body statically. Release keeps the pose frozen. |

Combine paused perturbation with the snapshot widget to capture keyframes by hand: pause, drag the body into position, hit **Save Snapshot**, then resume. The snapshot records the current `qpos` / `ctrl` — so the pose you manually dialled in replays exactly.

---

## Gizmos

While a drag is active `UMjPerturbation::DrawDebugSpring` overlays:

- **Red sphere** at the selection point on the body (follows the body through the drag).
- **Green arrow** during translate: from the selection point to the reference target. Length = position error the spring is solving.
- **Yellow arrow** during rotate: the tangent direction `ω × r` — the linear motion the rotation spring is trying to produce at the selection point. Naturally shrinks to zero as the body catches up, so it doesn't wobble when the rotation axis changes direction mid-drag.

---

## Implementation notes

- **Selection persists across drags.** Clicking with Ctrl held never reselects; only a plain double-click moves the selection. This avoids fat-finger re-selection during a rotate-then-translate sequence.
- **Cursor capture workaround.** UE's LMB and RMB capture freezes `GetMousePosition` / `GetInputMouseDelta` during a drag. To get real motion we register a Slate `IInputProcessor` (`FMjMouseDeltaProcessor`) that accumulates raw `FPointerEvent` deltas *before* Slate's capture layer zeroes them. Translate then rebuilds a virtual cursor screen position from the click pos + accumulated pixels and deprojects through that.
- **No force leak on release.** The pre-step callback zeros `d->xfrc_applied` every running step, matching simulate's contract — otherwise the last force would stay latched on the previously perturbed body and it would drift indefinitely.
- **Rotation clamp.** The reference quaternion is limited to ±90° relative to the body's current I-frame (same as `simulate.cc::mjv_movePerturb`). Without this the spring can stall or thrash if the ref drifts too far from reality.

---

## See also

- [Debug Visualization](debug_visualization.md) — hotkey reference, including the quick-convert wireframe overlay useful for diagnosing rest-pose issues.
- MuJoCo upstream: [`simulate.cc::mjv_movePerturb`](https://github.com/google-deepmind/mujoco/blob/main/simulate/simulate.cc) for the reference implementation of the gesture math.
