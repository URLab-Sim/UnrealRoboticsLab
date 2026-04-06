# ZMQ Networking & ROS 2

Unreal Robotics Lab uses [ZeroMQ](https://zeromq.org) for all external communication. The design philosophy: keep the Unreal side fast, binary, and dependency-free — then let a separate bridge handle translation to whatever framework the user prefers.

---

## Why ZMQ?

- **Speed.** Binary PUB/SUB with no serialization overhead. The control loop runs at physics-thread frequency (typically 500–1000 Hz) without bottlenecking on message encoding.
- **Minimal dependencies.** libzmq is the only networking dependency inside the plugin. No ROS, no middleware, no build system entanglement.
- **User's choice.** The plugin does not mandate a robotics framework. ZMQ is the native transport; if you want ROS 2, the separate `urlab_bridge` handles the translation. If you want raw Python, just use `pyzmq`. If you want something else entirely, the binary format is straightforward to parse.

---

## Plugin-Side Components

Three components handle the networking, all auto-created on the `AAMjManager` at BeginPlay if none exist:

| Component | Socket | Default Endpoint | Purpose |
|-----------|--------|-------------------|---------|
| `UZmqSensorBroadcaster` | PUB | `tcp://*:5555` | Publishes sensor data, joint state, twist, actions |
| `UZmqControlSubscriber` | SUB + PUB | `tcp://*:5556` (SUB), `tcp://*:5557` (info PUB) | Receives control vectors and gain updates; publishes actuator info JSON |
| `UMjCamera` (ZMQ mode) | PUB | per-camera endpoint | Streams rendered camera frames on a dedicated thread (uses `SCS_FinalToneCurveHDR` capture source, so Post Process Volumes are respected) |

All Unreal-side sockets use `zmq_bind()`. External clients use `zmq_connect()`.

### Timing

ZMQ callbacks run directly on the physics thread:

1. **PreStep** — `UZmqControlSubscriber` reads incoming commands → writes to `mjData.ctrl`
2. **mj_step()** — physics advances
3. **PostStep** — `UZmqSensorBroadcaster` reads `mjData` → publishes

One-timestep latency between command and response. Camera frames run on their own background thread so they never stall the physics loop.

### Topics (Sensor Broadcast)

All messages are multipart: topic string + binary payload. Topics are prefixed with the articulation actor name:

| Topic Pattern | Payload | Source |
|---------------|---------|--------|
| `{Name}/joint/{JointName}` | `int32 id, float pos, float vel, float acc` (16 bytes) | `UMjJoint::BuildBinaryPayload` |
| `{Name}/sensor/{SensorName}` | `int32 id, float[] values` (4 + 4*dim bytes) | `UMjSensor::BuildBinaryPayload` |
| `{Name}/base_state/{JointName}` | `7 x float64` (pos xyz + quat wxyz) | `UMjFreeJoint::BuildBinaryPayload` |
| `{Name}/twist` | `3 x float32`: vx, vy, yaw_rate | TwistController |
| `{Name}/actions` | `int32` bitmask | TwistController (only sent when non-zero) |

### Topics (Control Receive)

The control subscriber filters on `{Name}/control` and `{Name}/set_gains`:

| Topic Pattern | Payload |
|---------------|---------|
| `{Name}/control` | `int32 count`, then `count` x `(int32 actuator_id, float value)` |
| `{Name}/set_gains` | JSON: `{"joint_name": {"kp": float, "kv": float, "torque_limit": float}, ...}` |

### Info Broadcast (Port 5557)

The control subscriber periodically publishes a JSON discovery message on the info endpoint containing actuator names, IDs, ranges, and camera endpoints. This is sent frequently at startup (every 50 steps for the first 5s), then every 500 steps (~1s).

> **Multi-articulation filtering:** In multi-robot scenes, subscribe with the articulation name prefix (e.g., `sub.setsockopt_string(zmq.SUBSCRIBE, "Robot_A/")`) to receive only that robot's data.

### Control Source

`EControlSource` determines whether actuators respond to ZMQ or dashboard input. Set it globally on the manager or per-articulation:

```
Manager->SetControlSource(EControlSource::ZMQ);
```

---

## urlab_bridge (ROS 2)

The **urlab_bridge** (separate companion repository, same GitHub organization) is the Python-side middleware. It sits between the plugin's ZMQ streams and any external system (ROS 2, RL policies, custom scripts):

```
Unreal (ZMQ binary) → urlab_bridge → ROS 2 topics
```

It subscribes to the ZMQ sensor and camera endpoints, unpacks the binary payloads, and publishes to standard ROS 2 topics (`/joint_states`, `/sensor_data`, `/camera/image_raw`). Multi-robot namespacing is handled automatically.

**Why a separate bridge?**

- Keeps the Unreal plugin free of ROS build dependencies (ament, colcon, etc.)
- The bridge is pure Python with `pyzmq` + `rclpy` — easy to install, easy to modify
- Users who do not need ROS never have to think about it
- Users who do need ROS get standard topic interfaces without any plugin changes

### Quick test (no ROS needed)

```bash
uv run src/zmq_visualizer.py \
    --main_endpoint="tcp://127.0.0.1:5555" \
    --camera_endpoint="tcp://127.0.0.1:5558"
```

Prints live joint states and opens an OpenCV window for camera frames.

### ROS 2 rebroadcaster

```bash
# Source your ROS 2 environment first (Humble, Jazzy, etc.)
uv run src/ros2_broadcaster.py \
    --main_endpoint="tcp://127.0.0.1:5555" \
    --camera_endpoint="tcp://127.0.0.1:5558"
```

---

## Connecting from Python

Minimal `pyzmq` example:

```python
import zmq, struct, numpy as np

ctx = zmq.Context()

# Receive sensor data (Unreal binds, we connect)
sub = ctx.socket(zmq.SUB)
sub.connect("tcp://127.0.0.1:5555")
sub.setsockopt_string(zmq.SUBSCRIBE, "MyRobot/")

# Send controls (Unreal binds SUB on 5556, we connect PUB)
pub = ctx.socket(zmq.PUB)
pub.connect("tcp://127.0.0.1:5556")

while True:
    topic, data = sub.recv_multipart()
    topic_str = topic.decode()

    if "/joint/" in topic_str:
        jid, pos, vel, acc = struct.unpack("<Ifff", data)
        print(f"Joint {jid}: pos={pos:.3f}")

    # Send control: 3 actuators example
    num = 3
    payload = struct.pack("<I", num)  # count
    for i in range(num):
        payload += struct.pack("<If", i, 0.0)  # (id, value) pairs
    pub.send_multipart([b"MyRobot/control ", payload])
```

---

## Troubleshooting

**No data arriving** — check that endpoints match (protocol, IP, port). All Unreal-side sockets *bind*; external clients *connect*.

**Controls have no effect** — verify `ControlSource` is set to `ZMQ`. Note the control topic requires a trailing space in the subscription filter (e.g., `"MyRobot/control "`).

**Camera frames blank** — make sure `bEnableZmqBroadcast = true` on the camera component and the model compiled successfully.
