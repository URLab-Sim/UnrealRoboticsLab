# Python Overview

Drive URLab from Python with a typed client. Author scenes, run
Play-In-Editor (PIE), send control, read state and sensors, capture
cameras, and record or replay episodes, all over a ZMQ RPC session.

The client and everything around it ship from a separate companion
repo, `urlab_bridge`. Install it once, point it at a running editor,
and you have full programmatic control of the simulation.

![Python client driving a robot in Unreal](../images/placeholder.svg)

## Where to start

- New to the client: [Quickstart](quickstart.md) walks the full loop,
  connect, author a scene, run PIE, send control, step.
- Running pretrained policies: [Running Policies](policies.md) covers the
  bundled policies, the registry, and the gym wrapper.
- Method-level detail: [API Reference](api.md) documents `URLabClient`
  and every namespace.
- Wire format: [Protocol Reference](../reference/protocol.md) is the
  authoritative op + error catalogue.

## The bridge philosophy

URLab talks ZMQ first. The client speaks a msgpack RPC protocol over a
TCP REQ/REP socket; state and camera frames stream over PUB/SUB. This
keeps the core dependency-light and cross-host: you can run Python on
one machine and the editor on another. For same-host, latency-sensitive
control the client can use an optional shared-memory (SHM) transport
instead, carrying the same frames with lower tail latency. See
[Networking](../concepts/networking.md) for the transport details.

ROS 2 is a re-export, not the native layer. A separate broadcaster node
subscribes to the ZMQ state and camera streams and republishes them as
standard ROS 2 topics, so a ROS stack can consume URLab without linking
against the Python client. If you only need ROS topics, run the
broadcaster; if you want direct control, use `URLabClient`.

What ships from `urlab_bridge`:

- `urlab_client` - the typed client (`URLabClient` plus the `scene`,
  `sim`, `runtime`, `outliner`, `debug`, `viewport`, `recording`, and
  `replay` namespaces, and the `URLabArticulation` / `URLabEntity` /
  `URLabCameraView` wrappers).
- A DearPyGui dashboard (`urlab-ui`) for joint states, sensors,
  cameras, actuator sliders, recording, and a policy launcher.
- Bundled pretrained policies and the registry that exposes them.
- A ROS 2 broadcaster that bridges the ZMQ streams to ROS 2 topics.
- A `gymnasium.Env` wrapper so existing tools can drive URLab for evaluation and rollouts.

## Install the bridge

The bridge is a standalone repository that uses
[uv](https://docs.astral.sh/uv/). Clone it anywhere; it does not need
to live next to the URLab plugin. Python 3.11 or newer is required.

```bash
git clone https://github.com/URLab-Sim/urlab_bridge.git
cd urlab_bridge
uv sync
```

`uv sync` installs the core client. Add extras for the optional pieces:

| Goal | Command |
|---|---|
| Core client (`URLabClient`, transport, scene authoring) | `uv sync` |
| Dashboard | `uv sync --extra ui` |
| Bundled pretrained policies | `uv sync --extra robojudo` |
| mjlab integration | `uv sync --extra mjlab` |
| Dev tooling (pytest, ruff) | `uv sync --extra dev` |

!!! note "uv, not pip"
    The bridge is a uv project. Prefer `uv sync` and `uv run`. For a
    local checkout you depend on, add it under `[tool.uv.sources]`
    rather than reaching for `pip install -e`.

`mujoco` is part of the core install. The client loads the compiled
model the editor ships in the handshake, so the same MuJoCo build is
needed on both sides for `puppet` mode and any client-side IK / MPC.

## Connect

Start URLab and enter PIE, then open a session:

```python
from urlab_client import URLabClient

with URLabClient("tcp://localhost", step_mode="direct") as client:
    client.discover()
    print(client.urlab_version, client.mujoco_version)
    robot = client.articulations["robot"]   # keyed by MJCF model name
    for _ in range(1000):
        robot.set_ctrl({"shoulder": 0.5, "elbow": -0.2})
        client.step(n_steps=1)
```

See [Quickstart](quickstart.md) for the full flow.
