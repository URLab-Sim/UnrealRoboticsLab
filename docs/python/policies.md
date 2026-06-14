# Running Policies

URLab runs and evaluates policies, and you can train in it too. It steps one
simulation at a time, though, so for large-scale reinforcement learning a
dedicated massively parallel MuJoCo framework such as
[mjlab](https://github.com/mujocolab/mjlab) or
[mujoco_warp](https://github.com/google-deepmind/mujoco_warp) will train far
faster. Because they and URLab all use MuJoCo, a policy trained there evaluates
directly here, with Unreal's rendering, cameras, and recording.

This page covers running a bundled policy, registering your own, and wrapping a
client as a `gymnasium.Env` for evaluation pipelines.

If you only want to see a robot move, follow
[Run a bundled policy](#run-a-bundled-policy). To plug URLab into an evaluation
pipeline, jump to [Gym environment](#gym-environment).

![Bundled locomotion policy driving a humanoid](../images/placeholder.svg)

## Run a bundled policy

### 1. Install the policy extra

```bash
cd urlab_bridge
uv sync --extra ui          # dashboard
uv sync --extra robojudo    # bundled pretrained policies
```

PHC-flagged policies (BeyondMimic, AMO, H2H, twist_tracker) also need
the RoboJuDo submodule fully checked out:

```bash
cd RoboJuDo && git submodule update --init --recursive
```

### 2. Import the matching MJCF

Each policy expects a specific model. Drag the `.xml` into the Unreal
Content Browser; the importer runs the mesh pipeline and produces an
Articulation Blueprint.

| Key | Robot | DOF | MJCF |
|---|---|---|---|
| `unitree_12dof` | G1 | 12 | `assets/robots/g1/g1_29dof_rev_1_0.xml` |
| `unitree_wo_gait` | G1 | 29 | `assets/robots/g1/g1_29dof_rev_1_0.xml` |
| `smooth` | G1 | 29 | `assets/robots/g1/g1_29dof_rev_1_0.xml` |
| `beyondmimic_dance` | G1 | 29 | `assets/robots/g1/g1_29dof_rev_1_0.xml` |
| `amo` | G1 | 29 | `assets/robots/g1/g1_29dof_rev_1_0.xml` |
| `h2h` | G1 | 21 | `assets/robots/g1/g1_29dof_rev_1_0.xml` |
| `twist_tracker` | G1 | 12 | `assets/robots/g1/g1_29dof_rev_1_0.xml` |
| `go2_wtw` | Go2 | 12 | `mujoco_menagerie/unitree_go2/go2.xml` (download) |

The G1 XMLs ship in `urlab_bridge/assets/robots/g1/` with their mesh
dirs. Go2 comes from
[mujoco_menagerie](https://github.com/google-deepmind/mujoco_menagerie).

### 3. Add a PD controller

Every articulation driven by a bundled policy needs a `UMjPDController`
component on its Blueprint. The policies emit position targets at
policy rate; the PD controller converts those into per-step joint
torques against the live `qpos` / `qvel`. Without it the robot either
does not move or moves with the wrong gains.

1. Open the imported Articulation Blueprint.
2. Add Component, search `MjPDController`, add it. It auto-binds to
   every actuator.
3. Compile and save.

Tune gains in the Blueprint defaults, or at runtime from Python through
`art.controller.set_gains(...)` (see [Controllers](api.md#controllers)).

### 4. Run

Start UE, drop the Blueprint into a level, click Play. The dispatcher
comes up on `tcp://localhost:5559`.

From the dashboard:

```bash
uv run urlab-ui
```

Connect, open the Policy tab, pick a policy, set the articulation
prefix (for example `g1`), and click Run.

Headless from the CLI:

```bash
uv run urlab-policy --policy unitree_12dof --prefix g1
```

The launcher checks the policy's `required_step_mode` against the
current bridge mode and refuses to start in an incompatible one, naming
the mode it needs. Pass `--help` for the full flag set.

## The policy registry

The registry binds a short key to a config class, a URLab env config,
an MJCF asset, and a DOF count. The dashboard, the `urlab-policy`
launcher, and your own code all look policies up by name.

```python
from urlab_policy.registry import POLICIES, get_policy_labels

print(get_policy_labels())
print(POLICIES["unitree_12dof"]["dofs"])
```

### Adapters

The merged `urlab_policy.registry.POLICIES` is the union of every
adapter's `registry.py`. Two adapters ship:

- `adapters/robojudo/` - the RoboJuDo ecosystem. All the bundled
  entries above live here.
- `adapters/mjlab/` - mjlab-trained policies, run via the
  framework-agnostic `PolicyRunner` + `TaskSpec` machinery.

### Add a policy

Drop an entry into the relevant adapter's `registry.py`, copying an
existing one as a template. The schema is whatever the adapter reads;
open the adapter source to see the required fields. After it lands:

```bash
uv run urlab-policy --policy my_policy --prefix robot
uv run urlab-ui                            # picks it up in the Policy tab
```

The dashboard imports the merged registry on launch, so new entries
show up next session.

### `required_step_mode`

Mode-sensitive policies declare what they need; the launcher refuses to
start in an incompatible mode rather than producing silent garbage.

```python
"my_policy": {
    # ...
    "required_step_mode": "direct",              # single mode
    # or
    "required_step_mode": ("direct", "puppet"),  # any-of
}
```

| Policy trait | Declare |
|---|---|
| Calls `mj_step` on `client.data` itself (MJX, custom integrator) | `puppet` |
| Reads `art.controller` gains, expects UE to step | `direct` |
| Wraps a teleop device, needs continuous publishing | `live` |
| Pure black-box `act(obs) -> action` | omit (works everywhere) |

### Skip the registry for one-offs

The registry is for pretrained policies that should be discoverable.
For a one-off control loop, skip it and use `URLabClient` directly:

```python
from urlab_client import URLabClient

with URLabClient("tcp://localhost", step_mode="direct") as client:
    client.discover()
    client.sim.start()
    robot = client.articulations["g1"]
    for _ in range(1000):
        robot.set_ctrl({"left_hip_pitch": 0.5})
        client.step(n_steps=10)
```

## Gym environment

`URLabEnv` is a thin `gymnasium.Env` wrapper around a single
`URLabClient`. Use it to step a policy trained elsewhere through
URLab's contacts and rendering, or when a script already speaks the
gym interface.

!!! note "Not for high-throughput training"
    One process, one editor, one env. Vectorised GPU sims (mjlab, MJX,
    `mujoco_warp`) train orders of magnitude faster. Use `URLabEnv` for
    the rollout that matters: eval, sim-to-sim verification, or a
    sanity rollout against URLab's higher-fidelity contacts.

### Minimal usage

```python
from urlab_client import URLabClient
from urlab_policy.adapters.robojudo.env import URLabEnv

client = URLabClient("tcp://localhost", step_mode="direct")
client.discover()

env = URLabEnv(client)
obs, info = env.reset(seed=42)

for _ in range(1000):
    action = env.action_space.sample()
    obs, reward, terminated, truncated, info = env.step(action)
    if terminated or truncated:
        obs, info = env.reset()

env.close()
```

`URLabEnv` wraps an already-discovered client (its first positional
argument). The compiled model is at `env.client.model`, and the client
itself is reachable as `env.client`.

### Observation and action spaces

```python
URLabEnv(client, space_mode="flat")   # default
URLabEnv(client, space_mode="dict")
```

| Mode | Type | Use when |
|---|---|---|
| `flat` | `Box` (qpos + qvel + sensors concatenated in discovery order) | Single-articulation rollouts, flat actor-critic inputs |
| `dict` | `Dict` keyed by articulation prefix | Multi-articulation scenes, structure-aware nets |

### Rewards and termination

Both are opt-in callables; URLab does not invent rewards. Without them,
`reward` is `0.0` and `terminated` is `False` every step. Both take the
same `info` dict the env returns from `step()`, which carries the live
client, the per-articulation collections, `step_count`, and `sim_time`.

```python
def my_reward(info) -> float:
    art = info["client"].articulations["vx300s"]
    waist = art.qpos_array[art.joints["waist"].qpos_local_offset]
    return -abs(waist - 0.5)

env = URLabEnv(client, reward_fn=my_reward, max_episode_steps=1000)
```

### Seeding

```python
obs, info = env.reset(seed=42)
```

`reset(seed=...)` records the seed URLab-side (the manager's `Seed`
field) so client code and the recording layer can mirror it for
reproducibility.

!!! warning "The seed is not written into MuJoCo options"
    Modern `mjOption` has no seed field, and `mj_step` does not depend
    on a stored integrator seed; randomness comes from user-set noise
    inputs. In `direct` mode, the same seed plus the same scene and
    starting keyframe still reproduces a rollout because the integrator
    is deterministic, not because a seed was pushed into MuJoCo. In
    `puppet` mode the client owns the integrator, so reproducibility
    follows your local MuJoCo install.

### Observation level

```python
URLabEnv(client, observations="standard")   # default
URLabEnv(client, observations="minimal")
URLabEnv(client, observations="full")
```

Trades wire bandwidth for richness. See
[Protocol Reference](../reference/protocol.md#observation-levels) for
the per-level contents.

### Closing

```python
env.close()
```

Tears down the underlying client socket. Always call it; UE keeps the
session alive until the client disconnects.

## See also

- [Quickstart](quickstart.md) - the manual control loop.
- [API Reference](api.md) - controllers, articulations, step modes.
- [Protocol Reference](../reference/protocol.md) - the wire ops.
