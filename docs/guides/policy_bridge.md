# URLab Bridge

The URLab Bridge is a separate companion repository (`urlab_bridge`) in the same GitHub organization. It provides Python-side middleware between external systems and URLab's MuJoCo simulation, communicating over ZMQ for RL policy deployment, remote teleoperation, data logging, sensor monitoring, and custom control pipelines.

## What It Does

- Bidirectional ZMQ communication: receives joint state, sensor data, base state, and camera images from Unreal; sends actuator commands and PD gains back
- Remote control: drive robots from Python scripts, Jupyter notebooks, ROS 2 nodes, or any ZMQ-capable system
- Policy deployment: run pretrained RL policies (locomotion, motion imitation, quadruped gaits) against the Unreal simulation
- Sensor monitoring: real-time dashboard (DearPyGui) for visualizing joint states, sensor readouts, and camera feeds
- Handles articulation discovery, joint mapping, and actuator ID resolution automatically
- Provides forward kinematics for body position computation
- Prefix-based filtering for multi-articulation scenes
- Supports both position targets (for position actuators) and torque targets (for motor + PD)

## Architecture

```
┌─────────────────────────────────┐     ZMQ      ┌──────────────────┐
│  Python (URLab Bridge)          │◄────────────►│  Unreal (URLab)  │
│                                 │              │                  │
│  Your Control System            │  joint state │  MuJoCo physics  │
│    ├─ RL Policy (RoboJuDo)      │◄─────────────│  ZMQ Broadcaster │
│    ├─ Custom Controller         │              │                  │
│    └─ Teleoperation             │  ctrl targets│  ZMQ Subscriber  │
│                                 │─────────────►│  → d->ctrl       │
└─────────────────────────────────┘              └──────────────────┘
```

## Setup

The bridge is a **separate repository** — <https://github.com/URLab-Sim/urlab_bridge>. Clone it anywhere you like; it does not need to live next to the URLab plugin. Uses [uv](https://docs.astral.sh/uv/) for Python env management; requires Python 3.11+.

```powershell
git clone https://github.com/URLab-Sim/urlab_bridge.git
cd urlab_bridge
```

Minimum install (core dashboard + ZMQ — no policies):

```powershell
uv sync
```

That provisions a `.venv/`, pins Python 3.11+, and installs `pyzmq`, `numpy`, `opencv-python`, `dearpygui`.

For RL policy support (adds `torch`, `onnxruntime`, `mujoco`, etc.):

```powershell
uv sync --extra policy
uv pip install -e ./RoboJuDo
```

If the RoboJuDo install hangs (known uv/setuptools issue with its dynamic requirements resolution), use `--no-deps` since everything it needs is already in the `policy` extra, then add the two it uniquely needs:

```powershell
uv pip install --no-deps -e ./RoboJuDo
uv pip install pygame pynput
```

PHC-dependent policies additionally need:

```powershell
cd RoboJuDo && git submodule update --init --recursive
```

## ZMQ Protocol

For ZMQ protocol details, topic formats, and message structures, see [ZMQ Networking](zmq_networking.md).

## Using the Bridge Directly

For custom controllers without RoboJuDo:

```python
from urlab_policy.unreal_env import ZmqLink
import struct, time, numpy as np

zmq = ZmqLink("tcp://127.0.0.1:5555", "tcp://127.0.0.1:5556")
time.sleep(1)  # Wait for connection

# Read state
messages = zmq.drain()
for topic, payload in messages.items():
    if "/joint/" in topic and len(payload) == 16:
        jid, pos, vel, acc = struct.unpack("<Ifff", payload)
        print(f"Joint {jid}: pos={pos:.3f}")

# Send control (12 actuators, all to zero)
targets = np.zeros(12)
zmq.send_control("my_robot_prefix", targets)

zmq.close()
```

## Running

All three entry points go through `src/run.py`:

```powershell
# Dashboard (joint / sensor / camera viewer, actuator control, in-GUI policy runner)
uv run src/run.py --ui

# Headless policy loop
uv run src/run.py --policy unitree_12dof --prefix g1

# ZMQ smoke test — prints received joint messages for 10 s (no policy, no RoboJuDo)
uv run src/run.py --test --prefix g1
```

Start URLab in PIE (unpause MuJoCo) before running any of them.

Available policies are registered in `src/urlab_policy/policy_registry.py`. Each entry defines the policy config class, environment config, DOF count, and controller type. See the bridge's `README.md` for the current policy table.

### Go2 Walk-These-Ways Policy

12-DOF quadruped policy with gait conditioning. Supports preset gaits: Trot, Pronk, Bound, and Pace. Requires a checkpoint download (see `RoboJuDo/checkpoints/` README for links).

## Motor vs Position Actuators

| Approach | MJCF Actuator | Control Signal | PD Location |
|----------|--------------|----------------|-------------|
| Position | `<position kp="100" kv="5">` | Position target (rad) | MuJoCo internal |
| Motor + PD | `<motor forcerange="-88 88">` | Position target (rad) | C++ UMjPDController |

Position actuators are simpler and more stable. Motor + PD matches training dynamics exactly but requires gain configuration. Use the GUI checkbox to toggle.

## Configuration

Key settings in `env_config.py`:

| Field | Default | Must Match |
|-------|---------|------------|
| `sim_dt` | 0.002 | Unreal manager timestep |
| `sim_decimation` | 10 | Policy frequency = 1/(sim_dt * decimation) |
| `state_endpoint` | tcp://127.0.0.1:5555 | ZmqSensorBroadcaster endpoint (PUB) |
| `control_endpoint` | tcp://127.0.0.1:5556 | ZmqControlSubscriber endpoint (SUB) |

All Unreal-side sockets bind; the Python bridge connects. See [ZMQ Networking](zmq_networking.md) for topic formats and port assignments.

## Force Twist Override

The policy GUI provides a force twist override that lets you send manual velocity commands (vx, vy, yaw_rate) directly, bypassing keyboard possession input. Useful for scripted motion or testing without possessing the articulation in Unreal.

## Multi-Articulation Scenes

ZMQ topics use prefix-based filtering (`<prefix>/joint/...`, `<prefix>/control`, etc.). In scenes with multiple articulations, each one publishes and subscribes under its own prefix. The Python side filters incoming messages by prefix to route state to the correct policy instance.

## Mesh Preparation

Use `Scripts/clean_meshes.py` to convert meshes to GLB and resolve filename conflicts before import.

## Debugging Tools

The dashboard (`uv run src/run.py --ui`) is the primary diagnostic surface — it shows joint states, sensor readouts, camera feeds, and provides actuator sliders. For headless smoke-testing, `--test` prints raw ZMQ messages. For running a specific policy headlessly, `--policy`. See the bridge's `README.md` for the full flag list.
