# Entities

URLab scenes often contain dynamic objects that are not part of any articulation: boxes the robot pushes around, free-jointed pallets, heightfield terrain, standalone `UMjBody`s wired into the level. Remote stepping streams those alongside articulation state so your policy sees a complete world snapshot per step. They show up under one Python collection: `client.entities`.

> **Inspired by [mjlab's `Entity`](https://github.com/mujocolab/mjlab).**
> The `URLabEntity` / `URLabArticulation` split, the `EntityData`-style
> field names (`root_pos_w`, `root_quat_w`, `dof_qpos`, `dof_qvel`,
> `projected_gravity_b`, …), and the free-base / kinematic-body
> distinction all mirror mjlab. The benefit: mjlab obs / action terms
> read straight off a `URLabArticulation` without translation — see
> `urlab_policy/adapters/mjlab/runtime.py` for the wrapper. URLab's
> twist on it is that the same surface also serves the
> `URLabClient` (Python) and the underlying `AMjArticulation` (UE)
> sides of the bridge.

## What counts as an entity

At session start, URLab walks the scene for every dynamic `UMjBody`. Standalone (non-articulation) ones are bucketed into two kinds:

- **Free-jointed bodies**: a single free joint as their root. Have `qpos[7]` (pos + quat) and `qvel[6]` (linear + angular). Typical examples: props, pallets, free-flying sensors.
- **Kinematic dynamic bodies**: no free joint, but their world transform is driven by MuJoCo. Typical examples: `AMjHeightfieldActor` with a moving base, scene-root `UMjBody`s that inherit their transform.

Both kinds are enumerated once at `hello` and cached. Name collisions with articulation names are resolved by prepending `scene/` to the streaming topic.

## Accessing them from Python

Every dynamic body — articulations and standalone bodies alike — lives in `client.entities`. Articulations are a `URLabArticulation` (with extras); standalone bodies are bare `URLabEntity` instances.

```python
client.discover()

pallet = client.entities["pallet"]            # URLabEntity
pallet.root_pos_w                              # (3,) world-frame xpos
pallet.root_quat_w                             # (4,) MuJoCo wxyz
pallet.root_quat_xyzw                          # (4,) SciPy / ROS xyzw

pallet.has_free_base                           # True here
pallet.free_joint                              # "pallet_free"
pallet.free_joint_id                           # global MuJoCo joint id
pallet.qpos_offset                             # global qpos start (None if no free joint)
pallet.qvel_offset                             # global qvel start
```

`root_pos_w` / `root_quat_w` read from the local MJB mirror at the entity's `body_id`; the bridge writes the wire-shipped `xpos` / `xquat` into that slot every step. To read raw `qpos` / `qvel` slices for the free-joint bodies, index into `client.data` with the entity's offsets:

```python
import numpy as np
qpos = np.asarray(client.data.qpos[pallet.qpos_offset : pallet.qpos_offset + 7])
qvel = np.asarray(client.data.qvel[pallet.qvel_offset : pallet.qvel_offset + 6])
```

The surface mirrors `URLabArticulation` deliberately (which is itself a subclass of `URLabEntity`). The only real difference is grouping: articulations bundle a prefix of actuators plus joints plus sensors; standalone entities are one body each.

## What fields are populated

| Body kind | `root_pos_w` / `root_quat_w` | `qpos_offset` / `qvel_offset` |
|---|---|---|
| Free-jointed | always (from MJB mirror) | int into `client.data.qpos` / `qvel` (7 / 6 wide) |
| Kinematic dynamic | always | `None` |

`root_pos_w` and `root_quat_w` are always populated because the bridge writes the wire-shipped `xpos` / `xquat` into the local MJB mirror at the entity's `body_id` every step, regardless of whether a joint drives the body.

## Streaming behaviour per mode

| Mode | Transport |
|---|---|
| `live` | PUB on port 5555 under topic `scene/<name>/state`. Same binary layout as articulation topics. |
| `direct` | Parallel `entities` block in every step reply. |
| `puppet` | Already covered by the full `qpos` / `qvel` vector the client pushes; no extra block. |

In `puppet` the wrapper still exposes `client.entities[name]` for API consistency; it reads out of `client.data` after the push round-trip. The accessor shape is identical in all three modes.

## Example: reading entity state during a step

```python
from urlab_client import URLabClient, StepMode

client = URLabClient(step_mode=StepMode.DIRECT)
client.discover()

art = client.articulations["vx300s"]
pallet = client.entities["pallet"]

for _ in range(500):
    art.set_ctrl({"waist": 0.5})
    client.step(n_steps=5)

    if pallet.root_pos_w[2] < 0.1:
        print("pallet fell off the table")
        break
```

## `apply_xfrc`

Free-jointed entities accept external force / torque via `d->xfrc_applied`:

```python
pallet.apply_xfrc(force=[0.0, 5.0, 0.0], torque=[0.0, 0.0, 0.1])
```

Cleared after one step, same as articulation `apply_xfrc`. Safe in `live` and `direct`.

In `puppet` the call is **inert**. The client's `mj_step` is authoritative; anything UE applies on its side would be overwritten by the next push. The wrapper logs a one-line warning the first time `apply_xfrc` is called in puppet so you know the call did nothing. If you want external force in puppet, integrate it into your local `client.data` before pushing.

## Opt-out

There is no opt-out flag today. Entities stream by default in both `live` and `direct`. If bandwidth ever becomes an issue (many hundreds of dynamic props), a UPROPERTY toggle on `UURLabZmqPublishTransport` can turn it off; file an issue if you hit that case.
