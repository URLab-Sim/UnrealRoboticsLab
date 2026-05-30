# Step Server Wire Protocol

This page is the authoritative reference for the wire frames URLab speaks **during a live PIE session** — the handshake, the step / reset loop, recording / replay, and the runtime mutators (`set_mode`, `set_paused`, `set_qpos`, `set_mocap_pose`, `get_contacts`, etc.).

The **editor-only namespaces** (`scene.*`, `outliner.*`, `sim.*` for PIE lifecycle, `debug.*`, `viewport.*`) share the same envelope and msgpack encoding but aren't catalogued here — each op is documented in [Python API Reference](../python/api.md) through its `URLabClient` wrapper. Their wire names mirror the method names (`client.scene.spawn_actor` ↔ `op: spawn_actor`).

The [Networking guide](../guides/networking.md) is the user-facing overview of the whole transport story.

## Transports

The same wire shape is carried by two transports:

| Transport | Channels | Use |
|---|---|---|
| **ZMQ** | REQ/REP `tcp:5559` (RPC), PUB `tcp:5555` (state stream), PUB `tcp:5558+` (per camera) | Default. Cross-host. |
| **SHM** | `req.shm` / `rep.shm` (RPC), `state.shm` (state stream), `cam_<prefix>_<name>.shm` (per camera) | Same-host. Lower jitter. Falls back to ZMQ for oversize RPCs (e.g. `hello`). |

Files live under `<ProjectSavedDir>/URLabShm/<session_id>/` on the UE side. The bridge gets the absolute directory from the handshake (`shm_session_dir`).

## Wire encoding

Frames are **standard msgpack** by default. The Python bridge decodes with the C-extension `msgpack` package; UE encodes via a thin helper that walks an `FJsonObject` tree.

Every payload below is shown as JSON for readability; the actual frame is the equivalent binary msgpack.

### JSON fallback

For debugging, pass `"encoding": "json"` in the `hello` request:

```json
{
  "op": "hello",
  "client_version": "urlab_bridge/X.Y.Z",
  "observations": "standard",
  "encoding": "json"
}
```

The server then emits human-readable JSON wire frames for the rest of the session. Use this with `tcpdump`, `wireshark`, or a logging proxy.

## Session lifecycle

```mermaid
sequenceDiagram
    autonumber
    participant C as URLabClient
    participant S as Server (Dispatcher)

    C->>S: hello { observations, encoding }
    S-->>C: hello_ok { session_id, mjb, articulations[], shm_session_dir, ... }
    C->>C: load MJB into local MjModel/MjData
    opt step_mode != "auto"
        C->>S: set_mode { mode }
        S-->>C: set_mode_ok { current_mode }
    end
    loop policy
        C->>S: step { n_steps, observations, per_articulation }
        S-->>C: step_ok { time, sim_time, wall_time, per_articulation, ... }
    end
    C->>S: close — implicit; auto-revert to live
```

Every request after `hello` carries the returned `session_id`.

### `hello`

Request:

```json
{
  "op": "hello",
  "client_version": "urlab_bridge/X.Y.Z",
  "observations": "standard"
}
```

Reply:

```json
{
  "op": "hello_ok",
  "session_id": "uuid-v4",
  "urlab_version": "urlab/A.B.C",
  "mujoco_version": "3.7.0",
  "mjb": "<bytes>",
  "shm_session_dir": "C:/Users/.../Saved/URLabShm/live",
  "articulations": [
    {
      "prefix": "g1",
      "default_control_mode": "ue_controller",
      "controller": {
        "kind": "pd",
        "params": { "...": "..." },
        "schema": { "...": "..." }
      },
      "actuator_types": {
        "left_hip_pitch": "position",
        "right_hip_pitch": "position"
      },
      "camera_topics": {
        "head_rgbd": {
          "mode": "depth",
          "resolution": [848, 480],
          "fovy": 58.0,
          "zmq_endpoint": "tcp://*:5558",
          "zmq_topic": "g1/camera/head_rgbd"
        }
      }
    }
  ],
  "entities": {
    "pallet": {
      "id": 12,
      "has_free_base": true,
      "free_joint": "pallet_free",
      "free_joint_id": 3,
      "qpos_offset": 7,
      "qvel_offset": 6
    }
  },
  "global_cameras": {}
}
```

Notable fields:

- `mjb` — the compiled MuJoCo binary, shipped as msgpack `bin` (or base64 under JSON fallback). Loads via `mujoco.MjModel.from_binary_path` after a temp-file round-trip.
- `shm_session_dir` — absolute path to the SHM session directory. Bridge with `transport="shm"` opens `state.shm`, `req.shm`, `rep.shm`, `cam_*.shm` from here.
- `mujoco_version` — compare against `mujoco.__version__` and refuse to connect on mismatch. URLab pins MuJoCo via submodule.
- `camera_topics` — per-camera metadata. `mode` is `real` / `depth` / `semantic` / `instance`. A future PR will add an `intrinsics` block (fx, fy, cx, cy, near_cm, far_cm, distortion); see `docs/plan_camera_intrinsics.md` for the design.

The handshake only ships what MJB cannot carry: authored actuator kind (MuJoCo collapses `<position>` / `<velocity>` into `<general>`), controller identity + params + schema, and camera metadata. Joint names, ranges, offsets, gainprm, body xpos all round-trip through MJB.

## `step`

Advances physics and returns observations. Request shape varies with the active step mode.

### Direct variant

```json
{
  "op": "step",
  "session_id": "uuid-v4",
  "n_steps": 10,
  "observations": "standard",
  "include_cameras": false,
  "per_articulation": {
    "g1": {
      "ctrl": [0.5, -0.2, 0, 0, ...],
      "control_mode": "ue_controller",
      "xfrc_applied": {
        "left_foot": [0.0, 5.0, 0.0, 0.0, 0.0, 0.0]
      }
    }
  }
}
```

`control_mode` is optional; falls back to the articulation's `default_control_mode` from the handshake. `xfrc_applied` is cleared after one step. `n_steps` drives decimation: with `control_mode="ue_controller"`, UE's inner loop runs every sub-step against the fixed setpoint.

### Puppet variant

```json
{
  "op": "step",
  "session_id": "uuid-v4",
  "n_steps": 1,
  "observations": "standard",
  "include_cameras": false,
  "mode": "puppet",
  "time": 0.042,
  "qpos": ["..."],
  "qvel": ["..."],
  "ctrl": ["..."],
  "per_articulation": {}
}
```

`qpos` / `qvel` are full `m.nq` / `m.nv` vectors. `ctrl` is informational (stored for visualisation, not integrated). `xfrc_applied` is inert; UE logs a one-shot warning. `n_steps` is informational; UE calls `mj_forward` exactly once.

### Reply

Same shape for direct and puppet:

```json
{
  "op": "step_ok",
  "time": 0.042,
  "step": 21,
  "sim_time": { "sec": 0, "nsec": 42000000 },
  "wall_time": { "sec": 1714125000, "nsec": 123000000 },
  "per_articulation": {
    "g1": {
      "qpos": ["..."],
      "qvel": ["..."],
      "ctrl": ["..."],
      "act": ["..."],
      "sensors": { "imu_acc": ["..."], "imu_gyro": ["..."] },
      "twist": {
        "linear":  [0.0, 0.0, 0.0],
        "angular": [0.0, 0.0, 0.0]
      },
      "actions": 0
    }
  },
  "entities": {
    "pallet":    { "qpos": ["..."], "qvel": ["..."], "xpos": ["..."], "xquat": ["..."] },
    "terrain_a": { "xpos": ["..."], "xquat": ["..."] }
  }
}
```

Notable fields:

- `time` — `mjData->time` (legacy float seconds).
- `sim_time` / `wall_time` — ROS-Time-aligned `{sec, nsec}` blocks. `sim_time` mirrors `time` at nanosecond precision; `wall_time` is the publish moment in unix epoch. Use these for `header.stamp` in any ROS rebroadcaster, and for transport-latency measurement on the bridge side.
- `per_articulation[*].twist` / `actions` — only present when the articulation has a `UMjTwistController` attached (auto-spawned for every `AMjArticulation`). Maps directly to `geometry_msgs/Twist` plus a discrete-action bitfield.

In the puppet variant the reply echoes the pushed `qpos` / `qvel` so consumers can build a canonical observation dict without branching on mode.

### Camera observations

When `include_cameras` is non-false, the reply gains a `cameras` block:

```json
"cameras": {
  "head_rgbd": {
    "width": 848,
    "height": 480,
    "dtype": "float32",
    "data": "<bytes>"
  },
  "wrist_rgb": {
    "width": 640,
    "height": 480,
    "dtype": "bgra8",
    "data": "<bytes>"
  }
}
```

`dtype` is `bgra8` for Real / Semantic / Instance modes (4 bytes per pixel, BGRA byte order — the bridge swaps to RGBA on receive for Real, keeps BGRA for seg modes so consumers can map color → class id directly). `dtype` is `float32` for Depth (single channel, scene units / cm).

`include_cameras` accepts `true` (every camera, latest cached frame), `false` (none), or a per-camera dict like `{"head_rgbd": "sync", "wrist_rgb": "latest"}`. `"sync"` blocks the step until UE has captured a fresh frame; `"latest"` returns whatever's already there.

## `reset`

```json
{
  "op": "reset",
  "session_id": "uuid-v4",
  "keyframe_name": "home",
  "seed": 42,
  "per_articulation_qpos": {
    "g1": { "left_hip_pitch": 0.3 }
  }
}
```

All three override fields are optional. `seed` writes to `m->opt.seed`. `keyframe_name` resolves via `mjs_findKeyframe` and applies `qpos` + `ctrl` from it. `per_articulation_qpos` overrides specific joints by name. Server calls `mj_resetData`, applies overrides, then `mj_forward` once.

Reply shape matches `step_ok` minus `per_articulation[*].ctrl`.

## `set_mode`

Switch between `live`, `direct`, `puppet` mid-session. Only allowed when `AMjManager.StepMode == Auto`.

```json
{ "op": "set_mode", "session_id": "uuid-v4", "mode": "direct" }

{ "op": "set_mode_ok", "previous_mode": "live", "current_mode": "direct" }
```

Transitions flush pending writes. Any-mode-to-`live` resumes publishers; `live`-to-stepped pauses them within one tick. Direct↔puppet runs `mj_resetData` between them.

## `set_paused`

```json
{ "op": "set_paused", "session_id": "uuid-v4", "paused": true }

{ "op": "set_paused_ok", "paused": true }
```

Pauses / resumes physics. Publishers continue ticking on the most recently captured state.

## `set_sim_speed`

```json
{ "op": "set_sim_speed", "session_id": "uuid-v4", "percent": 25.0 }

{ "op": "set_sim_speed_ok", "percent": 25.0 }
```

Sets the engine's wall-clock pacer speed. 100 = realtime, 50 = half speed, etc. The engine clamps to `[5, 100]` internally; the reply echoes the effective value so the caller sees what stuck. Affects live pacing only — direct/puppet handlers run at the policy's cadence.

## `set_control_source`

```json
{ "op": "set_control_source", "session_id": "uuid-v4", "source": "zmq" }

{ "op": "set_control_source_ok", "source": "zmq", "scope": "global" }
```

Flips the actuator-read source between `"zmq"` (NetworkValue, what the bridge writes) and `"ui"` (InternalValue, what the editor's Details panel writes). Without `"articulation"` in the payload it applies globally (engine + every articulation); with it, only that articulation flips:

```json
{ "op": "set_control_source", "session_id": "uuid-v4", "source": "ui", "articulation": "g1" }

{ "op": "set_control_source_ok", "source": "ui", "scope": "articulation", "articulation": "g1" }
```

Useful for handing one articulation back to the editor mid-session (e.g. manual gain tuning) while the rest stay bridge-driven.

## `set_twist`

```json
{
  "op": "set_twist",
  "session_id": "uuid-v4",
  "articulation": "go2",
  "linear":  [0.4, 0.0, 0.0],
  "angular": [0.0, 0.0, 0.5]
}

{
  "op": "set_twist_ok",
  "articulation": "go2",
  "linear":  [0.4, 0.0, 0.0],
  "angular": [0.0, 0.0, 0.5]
}
```

Injects directly into the articulation's `UMjTwistController`, bypassing the WASD/gamepad input path. Use for headless eval or scripted command sequences where you can't drive the editor keyboard. `linear[0:2]` map to vx/vy (m/s); `angular[2]` maps to yaw rate (rad/s); other slots are accepted but ignored. Errors with `no_twist_controller` if the articulation has no `UMjTwistController` attached.

## `configure_controller`

Patches the active `UMjArticulationController` on an articulation. Same UE path as the existing `{prefix}/set_gains` PUB/SUB topic.

```json
{
  "op": "configure_controller",
  "session_id": "uuid-v4",
  "articulation": "g1",
  "params": {
    "kp": { "left_hip_pitch": 320.0 },
    "default_kv": 6.0
  }
}

{
  "op": "configure_controller_ok",
  "articulation": "g1",
  "params": {
    "kp":           { "left_hip_pitch": 320.0, "...": "..." },
    "kv":           { "left_hip_pitch": 20.0,  "...": "..." },
    "torque_limit": { "left_hip_pitch": 35.0,  "...": "..." },
    "default_kp": 100.0,
    "default_kv": 6.0,
    "default_torque_limit": 200.0
  }
}
```

Partial payloads are allowed; unmentioned fields are untouched. `set_defaults` rebinds every joint to the new default. See [Controllers and Gains](../python/controllers_and_gains.md) for the full schema semantics.

## `set_sim_options`

Pushes overrides into `m_model->opt`. Partial payloads override only the fields you list — same `bOverride_*` semantics as the editor's `FMuJoCoOptions` struct.

```json
{
  "op": "set_sim_options",
  "session_id": "uuid-v4",
  "options": {
    "timestep": 0.002,
    "gravity": [0.0, 0.0, -9.81],
    "integrator": "implicitfast"
  }
}

{
  "op": "set_sim_options_ok",
  "options": {
    "timestep": 0.002,
    "gravity":  [0.0, 0.0, -9.81],
    "wind":     [0.0, 0.0, 0.0],
    "magnetic": [0.0, -0.5, 0.0],
    "density": 0.0, "viscosity": 0.0, "impratio": 1.0,
    "tolerance": 1.0e-8, "iterations": 100, "ls_iterations": 50,
    "integrator": "implicitfast", "cone": "pyramidal", "solver": "newton",
    "noslip_iterations": 0, "noslip_tolerance": 1.0e-6,
    "ccd_iterations": 0, "ccd_tolerance": 1.0e-6,
    "enable_multiccd": false,
    "enable_sleep": false, "sleep_tolerance": 1.0e-4
  }
}
```

All values use MuJoCo-native SI: timestep in seconds, gravity / wind in m/s², magnetic in gauss. The reply echoes the live `m_model->opt.*` after apply, so you can verify the override took.

Field names accepted: `timestep`, `gravity`, `wind`, `magnetic`, `density`, `viscosity`, `impratio`, `tolerance`, `iterations`, `ls_iterations`, `integrator` (`euler` | `rk4` | `implicit` | `implicitfast`), `cone` (`pyramidal` | `elliptic`), `solver` (`pgs` | `cg` | `newton`), `noslip_iterations`, `noslip_tolerance`, `ccd_iterations`, `ccd_tolerance`, `enable_multiccd`, `enable_sleep`, `sleep_tolerance`.

Best called once after `hello` and before `step`. The live-mode pacer reads `opt.timestep` per loop iteration so changing it mid-session works, but you don't want a step in flight when the timestep flips.

## Recording

Recording hooks `OnPostStep` on the physics engine, so it captures one frame per `mj_step` regardless of mode.

```json
{ "op": "recording_start", "session_id": "uuid-v4",
  "name": "ep_1234", "max_duration_s": 3600.0 }
{ "op": "recording_start_ok", "name": "ep_1234", "max_duration_s": 3.4e38 }

{ "op": "recording_stop", "session_id": "uuid-v4" }
{ "op": "recording_stop_ok", "name": "ep_1234",
  "frame_count": 2048, "sim_duration_s": 4.096 }

{ "op": "recording_save", "session_id": "uuid-v4", "path": "ep_1234.json" }
{ "op": "recording_save_ok",
  "absolute_path": "C:/.../Saved/URLab/Replays/ep_1234.json" }

{ "op": "recording_clear", "session_id": "uuid-v4" }
{ "op": "recording_clear_ok" }
```

Save-path rules: bare filenames resolve under `<Project>/Saved/URLab/Replays/`, absolute paths pass through verbatim, and the reply always returns the resolved absolute path.

## Replay

Plays back a saved recording through the same custom-step-handler path used by `AMjReplayManager`. Requires `EStepMode` to be `direct` or `puppet`.

```json
{ "op": "replay_load", "session_id": "uuid-v4", "path": "ep_1234.json" }
{ "op": "replay_load_ok", "name": "ep_1234" }

{ "op": "replay_list_sessions", "session_id": "uuid-v4" }
{ "op": "replay_list_sessions_ok", "sessions": ["live", "ep_1234"] }

{ "op": "replay_set_active", "session_id": "uuid-v4", "name": "ep_1234" }
{ "op": "replay_set_active_ok" }

{ "op": "replay_start", "session_id": "uuid-v4" }
{ "op": "replay_start_ok", "active_session": "ep_1234", "total_frames": 2048 }

{ "op": "replay_stop", "session_id": "uuid-v4" }
{ "op": "replay_stop_ok" }
```

While replay is active, `step` requests advance the replay timeline instead of calling `mj_step`. `ctrl` and `xfrc_applied` in the step request are ignored; the reply adds `"replay": {"frame": N, "total_frames": M}`.

After `replay_stop`, the next `step` resumes live `mj_step` calls. `sim_time` continues forward from wherever the replay left off; issue a `reset` if you want a clean slate.

## Errors

Any failed request returns:

```json
{ "op": "error", "code": "...", "message": "..." }
```

| Code | Meaning |
|---|---|
| `version_mismatch` | Client's MuJoCo version disagrees with UE's `mj_version()`. |
| `session_expired` | `session_id` does not match the active session. |
| `unknown_articulation` | Prefix in the request does not resolve to any loaded articulation. |
| `ctrl_size_mismatch` | `ctrl` length does not match the articulation's actuator count. |
| `unknown_keyframe` | `keyframe_name` does not resolve via `mjs_findKeyframe`. |
| `model_changed_reconnect_required` | UE recompiled mid-session; client must re-handshake. |
| `mode_locked_by_server` | `set_mode` issued while `AMjManager.StepMode != Auto`. |
| `mode_mismatch` | `step` issued in a mode that does not accept it (e.g. a puppet-shaped step in direct mode). |
| `no_twist_controller` | `set_twist` targeted an articulation that has no `UMjTwistController` attached. |
| `replay_requires_stepped` | `replay_start` issued while in `live`. |
| `replay_session_not_found` | `set_active` / `start` named a session that is not loaded. |
| `recording_not_active` | `stop` / `save` issued without an active recording. |
| `recording_already_active` | `start` issued while another recording is running. |
| `path_not_writable` | `recording_save` target path is not writable. |
| `path_not_readable` | `replay_load` source path is not readable. |
| `no_controller` | `configure_controller` targeted an articulation without a `UMjArticulationController`. |
| `controller_schema_violation` | `params` failed the controller kind's schema validation. |
| `reply_too_large` | (SHM transport) reply exceeds the slot stride; bridge auto-falls back to ZMQ. |
| `wrong_transport` | (SHM transport) op is editor-only and not exposed over the SHM channel; bridge auto-falls back to ZMQ. |

## Observation verbosity

The `observations` field on `hello` and `step` selects how much payload UE puts on the wire. Server pays the cost of what is requested.

| Level | Content |
|---|---|
| `minimal` | `qpos`, `qvel`, `time`, `sim_time`, `wall_time`, `step` |
| `standard` | `minimal` + `act` + `ctrl` + `sensors` by name + `twist` + `actions` |
| `full` | `standard` + body `xpos` / `xquat` + actuator forces |

## SHM-only details

When the bridge connects with `transport="shm"`:

1. Initial `hello` round-trip goes over **ZMQ** (the MJB usually exceeds the SHM slot stride).
2. The handshake's `shm_session_dir` tells the bridge where the regions live.
3. Bridge swaps to a `ShmTransport` for subsequent ops; ZMQ is retained as a per-op fallback.
4. Windows-only: kernel-event signalling (`Local\URLab_<sid>_req_ready`, `Local\URLab_<sid>_rep_ready`) replaces polling for sub-millisecond RPC latency. Linux falls back to polling at `poll_interval_s` (default 1 ms).
5. Camera frames stream through `cam_<prefix>_<name>.shm` files, one per camera, at native resolution. Same `[u32 size][bytes]` slot layout as state.shm.
6. RPCs whose reply doesn't fit in the slot return `error(code="reply_too_large")`; the bridge auto-routes that op through ZMQ and remembers the choice for the rest of the session.

The wire layout is otherwise identical to ZMQ — same msgpack frames, same field shapes.
