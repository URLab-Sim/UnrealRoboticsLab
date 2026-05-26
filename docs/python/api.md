# URLab Python API Reference

`urlab_client` is the Python client for the URLab Unreal plugin
(shipped from the [urlab_bridge](https://github.com/URLab-Sim/urlab_bridge)
repository). It opens a ZMQ session against a running editor (or
PIE/standalone) and exposes the simulation, scene, and runtime through
typed Python objects.

This page is the reference. For a guided walkthrough of the common flows,
read [Python Getting Started](getting_started.md) first.

---

## Contents

- [Connection](#connection)
- [Transports](#transports)
- [Session state](#session-state)
- [Stepping the sim](#stepping-the-sim)
- [Namespaces](#namespaces)
  - [`client.scene`](#clientscene-editor-time-authoring) — editor-time authoring
  - [`client.sim`](#clientsim-pie-lifecycle) — PIE lifecycle
  - [`client.runtime`](#clientruntime-live-physics-mutators) — live physics mutators
  - [`client.outliner`](#clientoutliner-world-introspection) — world introspection
  - [`client.debug`](#clientdebug-debug-visualisation) — DrawDebug primitives + on-screen text
  - [`client.viewport`](#clientviewport-perspective-viewport-control) — perspective viewport camera
  - [`client.recording`](#clientrecording) / [`client.replay`](#clientreplay)
- [Articulations and entities](#articulations-and-entities)
- [Per-kind handles](#per-kind-handles)
- [Controllers](#controllers)
- [Cameras](#cameras)
- [Scene composition helpers](#scene-composition-helpers)
- [Typed return objects](#typed-return-objects)
- [Enums](#enums)
- [Errors](#errors)

---

## Connection

```python
from urlab_client import URLabClient

client = URLabClient("tcp://localhost", step_mode="direct")
client.discover()
...
client.close()
```

`URLabClient` is a context manager; `with URLabClient(...) as client:` closes
the transport on exit.

For new code on the same host, prefer `transport="shm"` —
[Transports](#transports) covers the trade-offs.

### `URLabClient(...)`

```python
URLabClient(
    address: str = "tcp://localhost",
    *,
    step_mode: str | StepMode = "auto",
    step_port: int = 5559,
    state_port: int = 5555,
    ctrl_port: int = 5556,
    info_port: int = 5557,
    mujoco_version_check: bool = True,
    local_model: bool = True,
    rcv_timeout_ms: int = 5000,
    auto_promote_step_mode: bool = True,
    transport: str | Transport = "zmq",
    shm_dir: str | None = None,
)
```

- `address` — TCP/IPC base address. Ports below are appended.
- `step_mode` — `"auto"` (default), `"direct"`, `"puppet"`, `"live"`,
  or a [`StepMode`](#stepmode) member. See [Stepping the sim](#stepping-the-sim).
- `step_port` (5559) — REQ/REP RPC.
- `state_port` (5555) — PUB/SUB state stream (used in `live`).
- `ctrl_port` (5556) — PUB/SUB live-control channel (reserved).
- `info_port` (5557) — PUB/SUB info channel (reserved).
- `transport` — `"zmq"` (default) or `"shm"` for shared-memory after the
  handshake. See [Transports](#transports).
- `local_model` — load the handshake's MJB into a local `mujoco.MjModel`
  on the client. Required for `puppet` mode and for client-side MPC / IK.
- `mujoco_version_check` — raise `URLabVersionMismatch` if server and
  client MuJoCo versions differ.
- `auto_promote_step_mode` — when `True`, `discover()` follows up with a
  `set_mode` RPC if the constructor specified `direct` or `puppet`.
- `rcv_timeout_ms` — default RPC recv timeout. Individual ops can override.

### `client.discover(observations="standard")`

Sends the `hello` handshake, loads the MJB, builds the articulation /
entity / camera wrappers. After this returns, all the collections below
are populated.

`observations` is one of:

- `"minimal"` — qpos, qvel only.
- `"standard"` — qpos, qvel, sensors. Default.
- `"full"` — adds per-body `xpos` / `xquat` and actuator forces.

`discover()` is safe to call multiple times. After `client.sim.start()`
returns `state="ready"`, the embedded handshake is auto-absorbed; you only
need to call `discover()` again if you want to change `observations`.

### `client.close()`

Stops streaming threads, reverts the server step mode (if auto-promoted),
closes the transport. Idempotent.

---

## Transports

`URLabClient` ships two transports. The default suits cross-host work;
the alternative cuts tail latency on the same machine.

### ZMQ (default)

```python
client = URLabClient("tcp://localhost", transport="zmq")
```

TCP REQ/REP on port 5559 for RPCs plus PUB/SUB on 5555 (state) and 5558+
(per-camera) for streams. Cross-host capable — point `address` at any
reachable host. This is the only path that works between machines.

### SHM (same-host)

```python
client = URLabClient("tcp://localhost", transport="shm")
```

Shared-memory ring buffers with kernel-event signalling. Lower jitter
than ZMQ, especially for camera streams. The `shm_session_dir` field in
the `hello` reply tells the bridge which directory under
`<ProjectSavedDir>/URLabShm/` carries the rings — no manual path
plumbing.

The `hello` handshake itself, oversize RPCs (anything that doesn't fit
in the request ring), and any op the SHM transport doesn't recognise
fall back to the ZMQ REQ socket transparently. The transport stays
`"zmq"` for those calls and resumes SHM for the next op.

### Picking one

| | ZMQ | SHM |
|---|---|---|
| Cross-host | yes | no |
| Same-host RPC latency | ~600–700 µs median | ~600 µs median, tighter p99 |
| Camera-stream jitter | higher | noticeably lower |
| Setup | `transport="zmq"` (default) | `transport="shm"` |
| Fallback | n/a | falls back to ZMQ for hello / oversize |

For the headline numbers, see the perf table in
[Networking](../guides/networking.md#performance-ballpark).

---

## Session state

Read-only attributes set by `discover()`:

| Attribute | Type | Notes |
|---|---|---|
| `session_id` | `str` | RPC session GUID |
| `urlab_version` | `str` | server plugin version |
| `mujoco_version` | `str` | server-side MuJoCo build |
| `manager_present` | `bool` | `False` pre-PIE (editor-only ops valid), `True` once PIE is live |
| `model` | `mujoco.MjModel \| None` | local model (when `local_model=True`) |
| `data` | `mujoco.MjData \| None` | local sim state (mirrors server each step) |
| `articulations` | `Dict[str, URLabArticulation]` | keyed by prefix |
| `articulations_by_id` | `Dict[str, URLabArticulation]` | subset with non-empty `actor_id` |
| `entities` | `Dict[str, URLabEntity]` | every dynamic body (articulations included) |
| `global_cameras` | `Dict[str, URLabCameraView]` | scene-level cameras |
| `recording` | `URLabRecordingAPI` | see [`client.recording`](#clientrecording) |
| `replay` | `URLabReplayAPI` | see [`client.replay`](#clientreplay) |
| `step_count` | `int` | steps since `discover()` |
| `sim_time` | `float` | server sim clock |

Time-aligned clocks (populated each step):
`sim_time_sec`, `sim_time_nsec`, `wall_time_sec`, `wall_time_nsec`,
`recv_wall_time_ns`.

> **Lock:** state read from non-step threads must hold `client._data_lock`
> (a reentrant lock). The streaming SUB callback in `live` mode
> writes through this lock.

---

## Stepping the sim

```python
reply = client.step(n_steps=1, *, include_cameras=False, observations="standard")
reply = client.reset(keyframe_name=None, seed=None, per_articulation_qpos=None)
```

### Step modes

| Mode | What advances physics | Typical use |
|---|---|---|
| `direct` | UE steps `n_steps` times per RPC. Client buffers ctrl into the request. | Deterministic, repeatable rollouts; data collection; unit-style sim tests. |
| `puppet` | Client calls `mj_step` locally, pushes qpos/qvel to UE for rendering. | MJX integration, MPC, custom integrators, JAX/torch differentiable sim. |
| `live` | UE physics runs autonomously; the RPC just stamps ctrl and reads state. | Low-latency control loops, teleop, real-time playback. |
| `auto` | Defaults to `live`. | When in doubt. |

In `puppet`, `n_steps=0` is supported: skip local `mj_step` and just push
whatever is already in `client.data` (useful for MJX or hand-authored
states).

### Reset

```python
client.reset()                                    # default reset
client.reset(keyframe_name="home")                # reset to keyframe
client.reset(seed=42)                             # deterministic seeded reset
client.reset(per_articulation_qpos={"robot": [...]})  # per-articulation override
```

Returns the same payload shape as `step()`, and mirrors qpos/qvel back
into `client.data`.

---

## Namespaces

The client groups RPCs by topic. Editor-only ops live under
`client.scene.*`, `client.outliner.*`, `client.debug.*`, and
`client.viewport.*`; PIE control under `client.sim.*`; live mutators
under `client.runtime.*`.

> **`manager_present` invariant:** `client.scene.*`, `client.outliner.*`,
> `client.debug.*`, and `client.viewport.*` ops are valid editor-time.
> `client.runtime.*` requires PIE to be live (`manager_present == True`).
> `client.sim.start()` is the bridge.

### `client.scene` — editor-time authoring

```python
client.scene.create_level(name, *, force_overwrite=False) -> None
client.scene.load_level(name_or_path) -> None
client.scene.save_level() -> None
client.scene.import_xml(xml_path, *, force_reimport=False) -> URLabBlueprint
client.scene.spawn_actor(blueprint, actor_id, *, location=..., rotation_quat=..., rotation_euler=..., scale=...) -> URLabSpawnHandle
client.scene.spawn_light(kind="directional", actor_id="", *, location=..., rotation_euler=..., intensity=5000.0, color=...) -> URLabLightHandle
client.scene.destroy_actor(target, *, by_name=False) -> None
client.scene.set_actor_transform(target, *, by_name=False, location=None, rotation_quat=None, rotation_euler=None) -> None
client.scene.destroy_asset(asset_path) -> bool
client.scene.current_level() -> str
client.scene.ensure_manager() -> bool
client.scene.snapshot() -> SceneSnapshot
client.scene.duplicate_actor(target, new_actor_id, *, by_name=False, location=None) -> URLabSpawnHandle
client.scene.actor_hierarchy(target, *, by_name=False) -> ActorHierarchyNode
client.scene.spawn_grid(blueprint, base_actor_id, count_x, count_y, *, spacing=(1,1,0), origin=(0,0,0), rotation_quat=None, rotation_euler=None, scale=(1,1,1)) -> Dict[str, URLabSpawnHandle]
client.scene.apply_scene(level_name, assets, *, save=True) -> Dict[str, URLabSpawnHandle]
```

`snapshot()` returns a heavier [`SceneSnapshot`](#scenesnapshot-scenesnapshotactor) than
`outliner.list_actors()` — for each URLab actor it also carries the
articulation metadata (joint / actuator / sensor / camera names).

`duplicate_actor()` reuses the source actor's blueprint class and
returns a fresh [`URLabSpawnHandle`](#urlabspawnhandle). Default
placement offsets +1 m on X; pass `location=...` to override.

`actor_hierarchy()` walks the attachment tree rooted at the target
and returns an [`ActorHierarchyNode`](#actorhierarchynode) (recursive).

`spawn_grid()` is the batched form of `spawn_actor`. Cells go at
`origin + (i*spacing.x, j*spacing.y, 0)` in MJ metres; each cell gets
`actor_id = f"{base}_{i}_{j}"`. Idempotent per cell id; server caps
the grid at 1024 cells.

`import_xml` returns a [`URLabBlueprint`](#urlabblueprint) handle that
plugs straight into `spawn_actor(blueprint=...)` — no dict unpacking:

```python
bp = client.scene.import_xml("C:/path/to/robot.xml")
handle = client.scene.spawn_actor(blueprint=bp, actor_id="robot_0")
```

`spawn_actor(blueprint=...)` accepts a `URLabBlueprint` *or* a raw
class-path string (`"/Game/MJ/Robot.Robot_C"`).

`apply_scene` is a convenience composer: load (or create) the level,
import each unique XML, spawn one actor per [`URLabAsset`](#urlabasset),
save. **Idempotent** — calling it twice with the same `actor_id` updates
the existing actor's transform and light parameters in place rather than
spawning a duplicate. Multi-instance scenes still work because each
instance gets its own `actor_id`. See
[Scene composition helpers](#scene-composition-helpers).

`create_level` overrides the new level's default game mode to
`AGameModeBase` so PIE doesn't transitively pull in the project-default
game mode and its blueprint references. This avoids `begin_pie`
hanging on a "Blueprint Compilation Errors" modal when the project
holds unrelated broken content (e.g. Marketplace mannequin animation
BPs).

`import_xml(force_reimport=True)` explicitly destroys the existing
generated blueprint asset before re-importing. Without that step the
asset-tools factory hits the `FKismetEditorUtilities::CreateBlueprint`
duplicate-name assertion and crashes the editor. The new BP lands at
the same class path (the importer derives the name from the XML file
stem), so callers can hold onto an old `URLabBlueprint` reference and
the class-path is still valid after a re-import.

`target` arguments default to looking up by `actor_id`. Pass
`by_name=True` to look up by UE actor name instead.

### `client.sim` — PIE lifecycle

```python
client.sim.start(level_path=None, *, timeout_s=30.0, raise_on_failure=True) -> PIEStartResult
client.sim.stop() -> None
client.sim.status() -> PIEStatus
```

`start()` returns a typed [`PIEStartResult`](#piestartresult). The
default behaviour raises `URLabPIEError` on failure (compile error,
timeout). Pass `raise_on_failure=False` to receive a `PIEStartResult`
in any state and inspect `.state` / `.compile_error` yourself:

```python
result = client.sim.start()        # raises URLabPIEError on failure
if not result.is_ready:
    raise RuntimeError("unreachable")  # default branch raised already

# Opt-out form for callers that want to handle failures locally:
result = client.sim.start(raise_on_failure=False)
match result.state:
    case PIEState.READY:           ...
    case PIEState.COMPILE_FAILED:  print(result.compile_error)
    case PIEState.TIMEOUT:         ...
```

On `PIEState.READY`, `result.handshake_payload` carries the fresh
handshake the client has already absorbed — you do not need to call
`discover()` again.

`start()` is cheap on a healthy session: if no `level_path` is supplied
and PIE is already running with a ready manager, the server short-
circuits to `state=READY` without issuing a `RequestPlaySession` round
trip. Pass an explicit `level_path` to force a level switch + restart.

`status()` returns a [`PIEStatus`](#piestatus) (state enum +
`compile_error` + `sim_time`).

### `client.runtime` — live physics mutators

```python
client.runtime.set_paused(paused: bool) -> bool
client.runtime.set_sim_speed(percent: float) -> float          # 100 = realtime
client.runtime.set_control_source(source, *, articulation=None) -> None  # "zmq" | "ui"
client.runtime.set_twist(articulation, *, linear=(0,0,0), angular=(0,0,0)) -> None
client.runtime.set_qpos(target, qpos, *, by_name=False) -> None
client.runtime.set_mocap_pose(body, *, pos=None, quat=None) -> MocapPose
client.runtime.read_mocap_pose(body) -> MocapPose
client.runtime.get_contacts(*, body1=None, body2=None, geom1=None, geom2=None, max_contacts=64) -> ContactsResult
client.runtime.list_keyframes() -> List[KeyframeInfo]
client.runtime.set_sim_options(**opts) -> SimOptions           # timestep, gravity, ...
client.runtime.set_mode(mode) -> StepMode                      # switch step mode
```

`set_qpos` mirrors the assignment back into `client.data.qpos` so the
local `MjModel` matches the server immediately. `set_sim_options`
returns the resulting [`SimOptions`](#simoptions) (typed mirror of the
`mjOption` fields URLab exposes) and also mirrors them into
`client.model.opt`.

`set_mocap_pose` / `read_mocap_pose` operate on the compiled MJ body
name (URLab prefixes already applied — use `scene.snapshot()` if
unsure). Errors `not_mocap_body` if the body isn't flagged
`mocap="true"`. `pos` is MJ metres, `quat` is wxyz.

`get_contacts` snapshots the active MuJoCo contacts. Filters AND-match
exact compiled names. The reply caps at `max_contacts` (default 64)
with a `truncated` flag if hit. Per-contact `force` is the 6-vector
`[fx, fy, fz, tx, ty, tz]` from `mj_contactForce` in the contact frame.

`list_keyframes()` enumerates the `<keyframe>` entries compiled into
the model. Pair with `client.reset(keyframe_name=...)` to load.

### `client.outliner` — world introspection

```python
client.outliner.list_actors() -> List[ActorInfo]
client.outliner.list_blueprints() -> List[BlueprintInfo]
client.outliner.find_actors(*, class_filter=None, tag=None, name_prefix=None, in_pie=False) -> List[ActorInfo]
client.outliner.get_actor_bounds(target, *, by_name=False, components_only=False) -> ActorBounds
client.outliner.select_actor(target, *, by_name=False) -> None
client.outliner.add_quick_convert(target, *, by_name=False, static=False, complex_mesh=False, coacd_threshold=0.05, driven_by_unreal=False, friction=(1, 1, 1)) -> None
client.outliner.remove_quick_convert(target, *, by_name=False) -> None
```

See [`ActorInfo`](#actorinfo), [`ActorBounds`](#actorbounds), and
[`BlueprintInfo`](#blueprintinfo) for the field shapes.

`find_actors()` filters AND across set parameters. `class_filter`
accepts a UE class short name (`"AMjArticulation"`, `"AStaticMeshActor"`,
`"AMjBody"`, ...). `in_pie=True` searches the PIE world if running.

`get_actor_bounds()` returns AABB in MJ metres. `components_only=True`
restricts to colliding components (drops volumes / triggers).

### `client.debug` — debug visualisation

```python
client.debug.draw_marker(location, color, *, ttl=0.0, label=None, tag=None) -> None
client.debug.draw_line(from_, to, color, *, ttl=0.0, thickness=1.0, tag=None) -> None
client.debug.draw_box(center, half_extents, color, *, rotation_quat=None, ttl=0.0, tag=None) -> None
client.debug.draw_arrow(from_, to, color, *, ttl=0.0, thickness=1.0, arrow_size=None, tag=None) -> None
client.debug.draw_axes(location, *, rotation_quat=None, rotation_euler=None, scale=0.2, ttl=0.0, tag=None) -> None
client.debug.clear_markers(*, tag=None) -> None
client.debug.set_overlay_text(text, *, anchor="top_left") -> None
```

Works in editor and PIE — the plugin picks the PIE world if running,
else the editor world. All positions are MJ metres; colors are
`[r, g, b]` floats in `[0, 1]`. `ttl` is seconds: `0` = single frame
(default), positive = expires after that many seconds, `-1` =
persistent until `clear_markers()`.

`draw_axes()` draws an RGB triad (X=red, Y=green, Z=blue) at the
target transform. `draw_arrow()` is a `DrawDebugDirectionalArrow`
wrapper — `arrow_size` is the head length in MJ metres (defaults to
20 % of shaft length). Tagged clearing is not honoured in v1 — the
plugin always does a full flush.

`set_overlay_text()` writes via `GEngine->AddOnScreenDebugMessage`
with a stable key, so successive calls update in place rather than
stacking. `anchor` is accepted for forward compat but ignored (UE
uses fixed top-of-viewport positioning).

### `client.viewport` — perspective viewport control

```python
client.viewport.set_camera(location, *, rotation_quat=None, rotation_euler=None, fov=None) -> CameraPose
client.viewport.get_camera() -> CameraPose
client.viewport.frame_actor(target, *, by_name=False) -> CameraPose
client.viewport.set_mode(mode) -> str
client.viewport.track_actor(target, *, by_name=False, offset=None, smoothing=0.0) -> str
client.viewport.untrack() -> bool
```

All ops operate on the editor's most-recently-focused perspective
viewport. Editor-only — the play-window in PIE has its own camera and
isn't affected.

`set_camera()` / `get_camera()` use MJ metres for `location` and xyzw
(UE FQuat order) for `rotation_quat`. `rotation_euler` is
`(roll, pitch, yaw)` in degrees.

`frame_actor()` calls `FEditorViewportClient::FocusViewportOnBox` on
the target actor's component bounds; the reply carries the resolved
camera pose so callers can persist / reproduce it.

`set_mode(mode)` accepts `"lit"`, `"unlit"`, `"wireframe"`,
`"collision"`, `"reflections"`. `track_actor()` installs a per-tick
callback that lerps the camera toward
`actor.location + actor.rotation * offset`. Default `offset` is
2 m behind and 1 m above. `smoothing ∈ [0, 1)` — `0` snaps, `0.95`
barely moves. Last writer wins; call `untrack()` to clear (idempotent;
returns `True` if a tracker was active).

> Screenshot capture (`viewport.screenshot`) is intentionally not
> shipped in this phase; see `docs/plan_followups.md`.

### `client.recording`

```python
client.recording.start(name=None, max_duration_s=None) -> RecordingHandle
client.recording.stop() -> RecordingSummary
client.recording.save(path=None) -> Path
client.recording.clear() -> None
```

`RecordingHandle.name` is the assigned recording name; `save()` returns
an absolute `pathlib.Path`. See
[`RecordingHandle`](#recordinghandle) and
[`RecordingSummary`](#recordingsummary).

Properties: `is_active`, `frame_count`, `sim_duration`, `last_saved_path`.

### `client.replay`

```python
client.replay.load(path) -> ReplaySession
client.replay.list_sessions() -> List[str]                          # session names
client.replay.set_active(name) -> None
client.replay.start() -> ReplayStatus
client.replay.stop() -> None
client.replay.play(path_or_name, *, loop=False) -> ReplaySession   # load + set_active + start
```

See [`ReplaySession`](#replaysession) and [`ReplayStatus`](#replaystatus).

Property: `active_session: Optional[str]`.

---

## Articulations and entities

Every dynamic body lives in `client.entities`. Articulations (anything
imported from MJCF as a multi-jointed robot) also live in
`client.articulations`, keyed by **prefix** — the MJCF model name.

`URLabArticulation` is a subclass of `URLabEntity`. `URLabEntity` carries
the root-link state and external-wrench buffer; `URLabArticulation` adds
the per-kind handle dicts and per-DoF state.

### Common state (both classes)

| Attribute | Shape | Frame |
|---|---|---|
| `root_pos_w` | `(3,)` | world |
| `root_quat_w` | `(4,)` | MuJoCo `(w, x, y, z)` |
| `root_quat_xyzw` | `(4,)` | SciPy/ROS `(x, y, z, w)` |
| `root_lin_vel_w` | `(3,)` | world |
| `root_ang_vel_w` | `(3,)` | world (articulation only) |
| `root_ang_vel_b` | `(3,)` | body (articulation only, MuJoCo native) |
| `root_lin_vel_b` | `(3,)` | body (derived) |
| `projected_gravity_b` | `(3,)` | body (derived) |

`apply_xfrc(body=..., force=..., torque=...)` (articulation) or
`apply_xfrc(force=..., torque=...)` (entity) buffers a 6-DOF wrench on a
named body. The buffer clears after the next `step()`. `clear_xfrc()`
clears it manually.

> `apply_xfrc()` is inert in `puppet` mode: the bridge overwrites
> `d->xfrc_applied` from the local `MjData` each step. The client warns
> at the call site.

### Articulation surface

Per-kind dicts (each keyed by short, unprefixed name):

```python
art.actuators: Dict[str, Actuator]
art.joints:    Dict[str, Joint]
art.sensors:   Dict[str, Sensor]
art.bodies:    Dict[str, Body]
art.cameras:   Dict[str, URLabCameraView]
```

Identity:

```python
art.prefix          # MJCF model name, e.g. "robot"
art.name            # alias for prefix (entity-level)
art.actor_id        # bridge stable id; "" if not assigned
art.body_id         # root body's global MuJoCo id
art.has_free_base   # True if root joint is mjJNT_FREE
art.free_joint      # name of the free joint (or None)
art.free_joint_id   # global MuJoCo id of the free joint
art.control_mode    # ControlMode.UE_CONTROLLER | ControlMode.RAW
art.controller      # URLabController | None  (None when control_mode == RAW)
```

Flat per-step arrays:

```python
art.ctrl_array          # (n_actuators,)  user setpoint buffer
art.last_applied_ctrl   # (n_actuators,)  read-only echo of UE-applied ctrl
art.act_array           # (n_actuators,)  activation state for stateful actuators
art.qpos_array          # dense per-articulation qpos
art.qvel_array          # dense per-articulation qvel
art.twist_linear        # (3,)  ROS-Twist linear (zeros if no UMjTwistController)
art.twist_angular       # (3,)  ROS-Twist angular
art.twist               # (6,)  [linear; angular]
art.actions             # int   bitfield from UMjTwistController
art.dof_qpos            # joint qpos excluding free-base root
art.dof_qvel            # joint qvel excluding free-base root
```

Bulk getters / setters:

```python
art.set_ctrl({"shoulder": 0.5, "elbow": -0.2})    # partial; missing keys keep last value
art.get_ctrl() -> Dict[str, float]
art.get_qpos() -> Dict[str, float]                 # first component per joint
art.get_sensors() -> Dict[str, np.ndarray]
art.push_gains(joint_names, stiffness, damping=None, torque_limit=None) -> int
```

MuJoCo id helpers:

```python
art.mj("joint", "shoulder")        # global id, accepts short or "<prefix>_<name>"
art.mj_joint("shoulder")
art.mj_actuator("shoulder")
art.mj_sensor("imu_gyro")
art.mj_body("hand")
art.mj_camera("wrist_cam")
```

Name resolution (handles original-XML names):

```python
art.resolve_joint("shoulder_joint")     # -> "shoulder" or None
art.resolve_actuator("shoulder")        # -> live key or None
```

Use these when caller-supplied names came from policies trained against
the source MJCF — the bridge sometimes renames joints/actuators to
satisfy UE blueprint uniqueness rules.

---

## Per-kind handles

### `Actuator`

```python
a.name         # str
a.id           # global MuJoCo actuator id
a.type         # ActuatorType | None  (authored kind from handshake)
a.trn_type     # transmission type ("joint", "tendon", ...)
a.joint        # unprefixed joint name (if joint-driven)
a.ctrlrange    # (lo, hi) | None
a.forcerange   # (lo, hi) | None
a.gear         # np.ndarray | None
a.gainprm      # np.ndarray | None  (kp, kv, ...)
a.dynprm       # np.ndarray | None
a.kp           # float | None  (gainprm[0] if present)
a.kv           # float | None  (gainprm[1] if present)
a.force        # float | None  (per-step at observation level "full")

a.set_control(v: float)
a.value -> float
```

### `Joint`

```python
j.name              # str
j.id                # global MuJoCo joint id
j.jnt_type          # int (0=free, 1=ball, 2=slide, 3=hinge)
j.qpos_offset       # global offset in client.data.qpos
j.qpos_dim          # 7 / 4 / 1
j.qvel_offset       # global offset in client.data.qvel
j.qvel_dim          # 6 / 3 / 1
j.qpos_local_offset # local offset within art.qpos_array
j.qvel_local_offset # local offset within art.qvel_array
j.range             # (lo, hi) | None
j.body_id           # body this joint belongs to
```

### `Sensor`

```python
s.name           # str
s.id             # global MuJoCo sensor id
s.sensor_type    # mjtSensor enum int
s.offset         # offset in MjData.sensordata
s.dim            # output dimension
s.latest         # np.ndarray | None  (per-step from sensors block)
```

### `Body`

```python
b.name    # str
b.id      # global MuJoCo body id
b.xpos    # (3,) | None  (per-step)
b.xquat   # (4,) | None  (per-step at observation level "full")
```

---

## Controllers

`art.controller` is `None` when the articulation's handshake didn't
ship a controller block (typically a raw-passthrough articulation).
Otherwise it is a `URLabController` (or its `URLabPDController`
subclass for PD-tuned articulations).

### `URLabController`

```python
ctrl.articulation_prefix   # str
ctrl.kind                  # ControllerKind | str
ctrl.params                # live param dict
ctrl.schema                # validation schema

ctrl.configure(**kwargs) -> Dict[str, Any]   # validate + push
ctrl.refresh() -> Dict[str, Any]              # re-pull from server
```

### `URLabPDController` (subclass)

Partial-patch semantics: joints not mentioned keep their current value.

```python
ctrl.kp                    # Mapping[str, float]   live view
ctrl.kv                    # Mapping[str, float]
ctrl.torque_limit          # Mapping[str, float]

ctrl.set_gains(kp=None, kv=None, torque_limit=None) -> dict
ctrl.set_defaults(kp=None, kv=None, torque_limit=None) -> dict
```

---

## Cameras

A camera is a `URLabCameraView`. Articulation-local cameras live under
`art.cameras`; scene-level cameras under `client.global_cameras`.

```python
cam.name           # str
cam.mode           # CameraMode (real / depth / semantic / instance)
cam.resolution     # (W, H)
cam.fovy           # vertical FOV in degrees
cam.owner          # articulation prefix or None
cam.dtype          # uint8 (real / segm) or float32 (depth)
cam.latest_frame   # np.ndarray | None
cam.sim_time       # float | None
cam.frame_count    # int
```

Frames are populated:

- on demand when you call `client.step(include_cameras=...)`.
  `include_cameras` accepts `True` / `False` (all-or-nothing) or a
  mapping such as `{"head": "sync"}` to select specific cameras and
  control whether the server waits for a fresh readback (`"sync"`) or
  ships the latest already-completed frame (`"latest"`, the default
  for entries without an explicit mode). The mapping form is honoured
  in every step mode (`direct`, `puppet`, and `live`).
- continuously from per-camera SUB threads in `live` mode.

Frame layouts:

| Mode | Shape | Channels |
|---|---|---|
| `real` | `(H, W, 4)` | RGBA |
| `depth` | `(H, W)` | float32 metres |
| `semantic` / `instance` | `(H, W, 4)` | BGRA (per-class / per-instance tint) |

---

## Scene composition helpers

```python
from urlab_client import URLabAsset, URLabBlueprint, URLabSpawnHandle, URLabLightHandle
```

### `URLabAsset`

One MJCF file plus the placement of one actor that will be spawned from
it. `apply_scene` consumes a list of these.

```python
@dataclass
class URLabAsset:
    actor_id: str                                    # bridge handle
    xml: str                                         # absolute MJCF path
    location: Sequence[float] = (0, 0, 0)
    rotation_quat: Optional[Sequence[float]] = None  # (x, y, z, w)
    rotation_euler: Optional[Sequence[float]] = None # (roll, pitch, yaw) deg
    scale: Sequence[float] = (1, 1, 1)
```

### `URLabBlueprint`

Returned by `client.scene.import_xml(...)`. Identifies a UE Blueprint
class compiled from an MJCF — pass it straight into
`spawn_actor(blueprint=...)`.

```python
@dataclass
class URLabBlueprint:
    class_path: str       # full UE class path, e.g. "/Game/MJ/Robot.Robot_C"
    short_name: str       # MJCF model name, e.g. "robot"
    imported_now: bool    # True if this call triggered a fresh import,
                          # False if the blueprint already existed
```

`spawn_actor(blueprint=...)` accepts a `URLabBlueprint` or a raw
`class_path` string for callers that want to skip the import step
(e.g., spawning from an already-imported asset).

### `URLabSpawnHandle`

Returned by `spawn_actor` / `apply_scene`. Pre-PIE: identity only. After
PIE starts, call `handle.runtime(client)` to resolve the live
`URLabArticulation`.

```python
@dataclass
class URLabSpawnHandle:
    actor_id: str
    actor_name: str
    actor_path: str
    blueprint_class_path: str
    location: tuple = (0, 0, 0)
    rotation_quat: tuple = (0, 0, 0, 1)
    was_existing: bool = False        # True if spawn_actor found a live actor
                                      # with this actor_id and updated it in
                                      # place; False on a fresh spawn
    requires_pie_restart: bool = False

    def runtime(self, client) -> URLabArticulation | None: ...
```

### `URLabLightHandle`

Lights are UE actors only — no physics counterpart.

```python
@dataclass
class URLabLightHandle:
    actor_id: str
    actor_name: str
    actor_path: str
    kind: str
    location: tuple = (0, 0, 0)
    rotation_euler: tuple = (0, 0, 0)
    intensity: float = 5000.0
    color: tuple = (1, 1, 1)
    requires_pie_restart: bool = False
```

---

## Typed return objects

Namespace methods return typed dataclasses (not raw dicts). Import from
`urlab_client`:

```python
from urlab_client import (
    PIEStartResult, PIEStatus, PIEState,
    SimOptions, ActorInfo, BlueprintInfo,
    RecordingHandle, RecordingSummary,
    ReplaySession, ReplayStatus,
)
```

### `PIEStartResult`

Returned by `client.sim.start()`.

```python
@dataclass
class PIEStartResult:
    state: PIEState                     # READY | COMPILE_FAILED | TIMEOUT
    compile_error: str = ""             # populated when state == COMPILE_FAILED
    handshake_payload: dict | None = None
                                        # fresh handshake on READY; the client
                                        # has already absorbed it

    @property
    def is_ready(self) -> bool: ...     # state == PIEState.READY
```

`PIEState` is `READY`, `COMPILE_FAILED`, `TIMEOUT`, `OFF`, `COMPILING`.

### `PIEStatus`

Returned by `client.sim.status()`.

```python
@dataclass
class PIEStatus:
    state: PIEState
    compile_error: str = ""
    sim_time: float | None = None
```

### `SimOptions`

Returned by `client.runtime.set_sim_options(...)`. Typed mirror of the
`mjOption` fields URLab exposes — the values reflect what the server's
`m_model->opt` actually has after applying the patch. Every field is
`Optional`; only fields the server echoed for this call are non-`None`.

```python
@dataclass
class SimOptions:
    timestep: float | None = None                    # seconds
    gravity: tuple[float, float, float] | None = None       # m/s²
    wind: tuple[float, float, float] | None = None          # m/s
    magnetic: tuple[float, float, float] | None = None      # gauss
    density: float | None = None
    viscosity: float | None = None
    impratio: float | None = None
    tolerance: float | None = None
    iterations: int | None = None
    ls_iterations: int | None = None
    integrator: str | None = None        # "euler" | "rk4" | "implicit" | "implicitfast"
    cone: str | None = None              # "pyramidal" | "elliptic"
    solver: str | None = None            # "pgs" | "cg" | "newton"
    noslip_iterations: int | None = None
    noslip_tolerance: float | None = None
    ccd_iterations: int | None = None
    ccd_tolerance: float | None = None
    enable_multiccd: bool | None = None
    enable_sleep: bool | None = None
    sleep_tolerance: float | None = None
```

### `ActorInfo`

Element type of `client.outliner.list_actors()`.

```python
@dataclass
class ActorInfo:
    name: str                                   # UE actor name
    actor_class: str                            # UE class path
    actor_id: str                               # bridge stable id ("" if none)
    is_articulation: bool
    has_quick_convert: bool
    location: tuple[float, float, float]
    rotation_quat: tuple[float, float, float, float]
    label: str = ""
    is_static_mesh_actor: bool = False
    is_light: bool = False
    # Quick-Convert fields (populated only when has_quick_convert is True):
    static: bool | None = None
    complex_mesh: bool | None = None
    coacd_threshold: float | None = None
    driven_by_unreal: bool | None = None
    friction: tuple[float, float, float] | None = None
```

### `BlueprintInfo`

Element type of `client.outliner.list_blueprints()`.

```python
@dataclass
class BlueprintInfo:
    class_path: str        # full UE class path, e.g. "/Game/MJ/Robot.Robot_C"
    short_name: str        # MJCF model name, e.g. "robot"
```

### `ActorBounds`

Returned by `client.outliner.get_actor_bounds()`. All values in MJ
metres.

```python
@dataclass
class ActorBounds:
    actor_name: str
    min: tuple[float, float, float]
    max: tuple[float, float, float]
    center: tuple[float, float, float]
    extents: tuple[float, float, float]
```

### `SceneSnapshot` / `SceneSnapshotActor`

Returned by `client.scene.snapshot()`.

```python
@dataclass
class SceneSnapshot:
    level_path: str
    in_pie: bool
    actors: list[SceneSnapshotActor]

@dataclass
class SceneSnapshotActor:
    name: str
    actor_class: str
    actor_id: str
    label: str
    location: tuple[float, float, float]
    rotation_quat: tuple[float, float, float, float]
    tags: list[str]
    urlab: dict | None        # MJ metadata for AMjArticulation actors:
                              # {mj_class, joints, actuators, sensors, cameras}
```

### `ActorHierarchyNode`

Returned by `client.scene.actor_hierarchy()`. Recursive tree of the
attachment hierarchy rooted at the target.

```python
@dataclass
class ActorHierarchyNode:
    name: str
    actor_class: str
    location: tuple[float, float, float]
    children: list[ActorHierarchyNode]
```

### `MocapPose`

Returned by `client.runtime.set_mocap_pose()` / `read_mocap_pose()`.
``pos`` in MJ metres; ``quat`` is wxyz (MuJoCo native).

```python
@dataclass
class MocapPose:
    body: str
    pos: tuple[float, float, float]
    quat: tuple[float, float, float, float]
```

### `Contact` / `ContactsResult`

Returned by `client.runtime.get_contacts()`. `force` is the 6-vector
`[fx, fy, fz, tx, ty, tz]` from `mj_contactForce` in the contact frame.
`dist` is negative when penetrating.

```python
@dataclass
class Contact:
    geom1: str
    geom2: str
    body1: str
    body2: str
    pos: tuple[float, float, float]
    normal: tuple[float, float, float]
    dist: float
    force: tuple[float, float, float, float, float, float]

@dataclass
class ContactsResult:
    n_contacts: int
    truncated: bool          # True if max_contacts cap was hit
    contacts: list[Contact]
```

### `KeyframeInfo`

Element type of `client.runtime.list_keyframes()`. All MJ arrays in
compiled order: `qpos[nq]`, `qvel[nv]`, `ctrl[nu]`,
`mocap_pos[3*nmocap]`, `mocap_quat[4*nmocap]`.

```python
@dataclass
class KeyframeInfo:
    name: str
    time: float
    qpos: list[float]
    qvel: list[float]
    ctrl: list[float]
    mocap_pos: list[float]
    mocap_quat: list[float]
```

### `CameraPose`

Returned by `client.viewport.set_camera()` / `get_camera()` /
`frame_actor()`. `location` in MJ metres; `rotation_quat` is xyzw
(UE FQuat order); `rotation_euler` is `(roll, pitch, yaw)` degrees;
`fov` is horizontal degrees.

```python
@dataclass
class CameraPose:
    location: tuple[float, float, float]
    rotation_quat: tuple[float, float, float, float]
    rotation_euler: tuple[float, float, float]
    fov: float
```

### `RecordingHandle`

Returned by `client.recording.start()`.

```python
@dataclass
class RecordingHandle:
    name: str                              # assigned recording name
    max_duration_s: float | None = None    # echo of the cap (or override)
```

### `RecordingSummary`

Returned by `client.recording.stop()`.

```python
@dataclass
class RecordingSummary:
    frame_count: int
    sim_duration_s: float
```

### `ReplaySession`

Returned by `client.replay.load()` / `play()`. Note that
`client.replay.list_sessions()` returns a plain `List[str]` of session
names, not `ReplaySession` instances.

```python
@dataclass
class ReplaySession:
    name: str
    total_frames: int = 0
    source_path: pathlib.Path | None = None
```

### `ReplayStatus`

Returned by `client.replay.start()`.

```python
@dataclass
class ReplayStatus:
    active_session: str
    total_frames: int = 0
```

---

## Enums

All enums in `urlab_client.enums` are `str`-mixin enums: their members
JSON / msgpack as the wire string with no conversion glue.

### `StepMode`

`AUTO`, `DIRECT`, `PUPPET`, `LIVE`. See [Stepping the sim](#stepping-the-sim).

### `ControlMode`

`UE_CONTROLLER`, `RAW` — per-articulation, set in the handshake.

### `ActuatorType`

`MOTOR`, `POSITION`, `VELOCITY`, `INT_VELOCITY`, `DAMPER`, `CYLINDER`,
`MUSCLE`, `ADHESION`, `DC_MOTOR`. The *authored* kind from MJCF (MuJoCo
collapses these into `<general>` after compilation).

### `ControllerKind`

`PD`, `PASSTHROUGH`.

### `CameraMode`

`REAL`, `DEPTH`, `SEMANTIC`, `INSTANCE`.

### `CameraTiming`

`SYNC` (per-step capture), `LATEST` (async stream, latest cached frame).

### `ObservationLevel`

`MINIMAL`, `STANDARD`, `FULL`. Drives how much per-step data the server
serialises into the step reply.

### `SpaceMode`

`FLAT`, `DICT`. Used by env adapters built on top of the client.

### Helpers

```python
from urlab_client.enums import coerce, wire

coerce(StepMode, "direct")          # -> StepMode.DIRECT
coerce(StepMode, StepMode.AUTO)     # passthrough
wire(StepMode.PUPPET)               # -> "puppet"
```

---

## Errors

```python
from urlab_client import (
    URLabRPCError, URLabVersionMismatch, URLabPIEError,
)
```

- **`URLabRPCError(code, message, *, op=None)`** — server returned an
  `error` reply. `code` is short and stable (e.g.
  `"editor_only_op"`, `"compile_failed"`, `"unknown_op"`); `message` is
  free-form. Catch this when you want to fall through to a different op
  (the `apply_scene` composer does this for `load_level`).
- **`URLabVersionMismatch`** — raised by `discover()` when client and
  server MuJoCo versions differ. Pass `mujoco_version_check=False` to
  the constructor to downgrade to a warning.
- **`URLabPIEError(*, code, message, state)`** — raised by
  `client.sim.start()` when the editor failed to enter `Playing`. It is
  a subclass of `URLabRPCError`, so callers can catch either type.
  `.state` is a `PIEState` member (`COMPILE_FAILED`, `TIMEOUT`,
  `OFF`, …); `.code` is the short error code (e.g. `pie_compile_failed`);
  `.message` carries the Blueprint compile log when relevant. Pass
  `raise_on_failure=False` to `sim.start()` to receive a
  `PIEStartResult` instead — it carries the same `state` /
  `compile_error` / `handshake_payload` fields.

---

## See also

- [Python Getting Started](getting_started.md) — guided
  walkthrough of the common flows.
- [Networking](../guides/networking.md) — UE-side architecture.
- [Step Server Protocol](../reference/step_server.md) — wire-protocol
  reference.
- The `meta` RPC — call `client._rpc("meta", {})` to get a live list of
  every op the server exposes, with required fields and reply schemas.
  [`tools/gen_stubs.py`](https://github.com/URLab-Sim/urlab_bridge/blob/main/tools/gen_stubs.py)
  in the bridge regenerates the typed `.pyi` from this.
