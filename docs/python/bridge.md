# URLab Bridge

The Python middleware that talks to URLab over ZMQ (and optionally
shared memory). Ships from the
[`urlab_bridge`](https://github.com/URLab-Sim/urlab_bridge) repo.

For driving URLab from Python in your own code, start with
[Python Getting Started](getting_started.md) — `URLabClient` is the
canonical entry point. This page covers the *packaged* side of the
bridge: installation, the DearPyGui dashboard, the bundled console
scripts, and the ROS 2 broadcaster.

## What ships

- **`URLabClient`** — typed Python client for the URLab step server.
  Scene authoring (`client.scene.*`), PIE lifecycle (`client.sim.*`),
  runtime mutation (`client.runtime.*`), outliner queries
  (`client.outliner.*`), debug visualisation (`client.debug.*`),
  perspective viewport control (`client.viewport.*`), recording / replay
  (`client.recording.*` / `.replay.*`). Lives in the `urlab_client`
  package. See [API Reference](api.md).
- **Dashboard** — DearPyGui-based control panel for joint states,
  sensors, cameras, actuator sliders, recording, and the bundled
  pretrained-policy launcher. Lives in the `urlab_dashboard` package.
  Launch via `urlab-ui`.
- **Bundled policies** — Unitree G1 locomotion (12 / 29 DOF),
  Walk-These-Ways Go2, BeyondMimic, AMO, H2H, Smooth, etc. Registered
  via `urlab_policy/adapters/robojudo/registry.py` and discoverable
  through the dashboard's Policy tab. See
  [Policy Registry](policy_registry.md).
- **ROS 2 broadcaster** — `urlab_tools.ros2_broadcaster` republishes
  the streaming PUB/SUB layer as standard ROS 2 topics. See
  [ROS 2 bridge](#ros-2-bridge) below.
- **Gym environment** — `URLabEnv(gym.Env)` for users integrating
  URLab into a training pipeline (stable-baselines3, CleanRL,
  RSL-RL). See [Gym Environment](gym_environment.md).

## Architecture at a glance

```
┌───────────────────────────────────┐       ┌───────────────────────────────┐
│  Python (urlab_bridge)            │       │  Unreal (URLab plugin)        │
│                                   │       │                               │
│  URLabClient ────────────────────►│ 5559  │  UURLabZmqRpcTransport        │
│  (scene / sim / runtime / ...)    │  REQ  │      │                        │
│                                   │  REP  │      ▼                        │
│  URLabArticulation ◄──────────────│ 5555  │  FURLabRpcDispatcher          │
│  URLabCameraView                  │  PUB  │      │                        │
│                                   │  SUB  │      ▼                        │
│  urlab_dashboard (DearPyGui)      │ 5558  │  UMjPhysicsEngine             │
│                                   │  PUB  │      ▲                        │
│  urlab_tools/ros2_broadcaster ◄──►│       │      │                        │
│  (legacy ZMQ ↔ ROS 2 topics)      │  SHM  │  UURLabShmRpcTransport        │
└───────────────────────────────────┘ alt.  │  (same host, lower jitter)    │
                                            └───────────────────────────────┘
```

- **REQ/REP on 5559** — `URLabClient` RPCs (scene authoring, PIE
  control, set_qpos, recording, etc.).
- **PUB on 5555** — state stream consumed by `live` mode and by the
  ROS 2 bridge.
- **SUB on 5556** — control writes from anything talking the legacy
  PUB/SUB protocol.
- **PUB on 5558+** — per-camera frame stream.
- **SHM (optional, same-host)** — lower-jitter alternative for the
  RPC + state streams. Falls back to ZMQ for hello and oversize ops.

The UE-side architecture (dispatcher, transports, snapshot fan-out)
is in [Networking](../guides/networking.md).

## Setup

Separate repo. Clone anywhere; no need to live next to the URLab
plugin. Uses [uv](https://docs.astral.sh/uv/) for the Python env.
Requires Python 3.11+.

```bash
git clone https://github.com/URLab-Sim/urlab_bridge.git
cd urlab_bridge
```

### Install

Pick the extras you need; the core install is minimal.

| Goal | Command |
|---|---|
| Core client only (`URLabClient`, transport, scene authoring) | `uv sync` |
| Dashboard | `uv sync --extra ui` |
| Pretrained RoboJuDo policies | `uv sync --extra robojudo` + `uv pip install -e ./RoboJuDo` |
| mjlab integration | `uv sync --extra mjlab` |
| Dev (pytest, ruff) | `uv sync --extra dev` |

For RoboJuDo specifically: if `uv pip install -e ./RoboJuDo` hangs
on dynamic-requirements resolution, do:

```bash
uv pip install --no-deps -e ./RoboJuDo
uv pip install pygame pynput
```

PHC-dependent policies additionally need:

```bash
cd RoboJuDo && git submodule update --init --recursive
```

## Console scripts

`pip install`-ing the bridge wires four scripts into your PATH.

| Script | Purpose |
|---|---|
| `urlab-ui` | Launch the DearPyGui dashboard. |
| `urlab-policy --policy <name>` | Run a bundled pretrained policy headless. |
| `urlab-ping` | Open a session against the editor's REQ/REP server, print the handshake summary, exit. Smoke check. |
| `urlab-test` | Dump the legacy PUB/SUB state stream for 10 s. Diagnostic. |

`uv run src/run.py --ui` / `--policy` / `--ping` / `--test` works too
(thin shim, kept for the legacy README). Pass `--help` on any of the
scripts for the full flag list.

Start URLab in PIE first; the bridge needs the step server (port
5559) and the streaming PUB (port 5555) up.

## Dashboard

Launch with `urlab-ui`. Tabs (in order of typical use):

- **Connection** — pick an editor address, hit Connect, see the
  handshake summary. The current step mode is a coloured pill at
  the top.
- **Scene** — pre-PIE scene authoring: create / load level, import
  XML, spawn / destroy actors, quick-convert mesh actors, list
  blueprints. Anything `client.scene.*` can do.
- **Runtime** — articulation inspector + the live `set_qpos`,
  `set_twist`, `set_control_source`, `set_sim_options` controls.
- **Cameras** — every articulation's camera streams, both REAL and
  Depth / Semantic / Instance.
- **Recording** — start / stop / save / load recordings; trigger
  replays. Calls into `client.recording.*` and `client.replay.*`.
- **Policy** — pick a bundled policy from the registry, set the
  articulation prefix, hit Run. Mirrors what `urlab-policy --policy
  <name>` does from the CLI.
- **Debug** — manual step / reset, render-viewer, raw RPC inspector.

Adding a custom policy: see [Policy Registry](policy_registry.md).
The new entry shows up in the dashboard's Policy tab next launch.

## ROS 2 bridge

`urlab_tools/ros2_broadcaster/broadcaster.py` is a single-node ROS 2
bridge that consumes the legacy PUB/SUB streams and republishes
them as standard ROS 2 topics. Useful when you want to talk to
URLab from a ROS 2 stack without touching `URLabClient` Python.

```bash
# Source your ROS 2 environment first (Humble / Jazzy / etc.)
source /opt/ros/humble/setup.bash

# Then launch the bridge (URLab + PIE need to be running):
uv run python -m urlab_tools.ros2_broadcaster
```

The node (`urlab_ros2_bridge`) auto-discovers articulations as
their state arrives and spins up the matching publishers /
subscribers. Per-articulation routing:

| ZMQ source (URLab → bridge) | ROS 2 destination |
|---|---|
| `{prefix}/joint/{name}` | `/{prefix}/joint_states` (`sensor_msgs/JointState`) |
| `{prefix}/sensor/{name}` | `/{prefix}/sensor/{name}` (`std_msgs/Float64MultiArray`) |
| `{prefix}/camera/{name}` | `/{prefix}/camera/{name}/image_raw` (`sensor_msgs/Image`) |
| `/{prefix}/control` (`std_msgs/Float64MultiArray`, ROS 2 → bridge) | `{prefix}/control ` (binary control packet, ZMQ) |

Defaults: state SUB `tcp://127.0.0.1:5555`, control PUB
`tcp://127.0.0.1:5556`, camera SUB `tcp://127.0.0.1:5558`. Edit
`URLabBridge.__init__` in `broadcaster.py` for non-defaults.

**Current limitations** (worth knowing before you depend on it):

- `JointState.effort` carries acceleration, not torque — it's the
  third float in the wire payload.
- Control mapping uses array index as actuator id; an
  actuator-name → id resolver via the 5557 info channel isn't wired
  up yet.
- Camera resolution is hardcoded to 640 × 480; frames that don't
  match the exact byte count are dropped silently.
- One ROS node per bridge process — multi-robot is prefix-routed,
  not per-namespace-isolated.

For the underlying wire format, see [Networking →
Streaming wire format](../guides/networking.md#streaming-wire-format-pubsub).

## See also

- [Python Getting Started](getting_started.md) — the typed
  `URLabClient` flow end-to-end.
- [Python API Reference](api.md) — every method + return type.
- [Policy Registry](policy_registry.md) — adding bundled policies.
- [Gym Environment](gym_environment.md) — gymnasium wrapper for
  training-pipeline integration.
- [Networking](../guides/networking.md) — UE-side architecture
  + the legacy streaming wire format.
