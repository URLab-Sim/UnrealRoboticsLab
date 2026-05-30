# Policy Registry

The `urlab_bridge` ships a flat registry of pretrained policies that the
dashboard, the `run.py` launcher, and the bridge UI pick up by name.
Each entry binds a short key to a RoboJuDo config class, a URLab env
config, an MJCF asset, and DOF count.

To run an existing pretrained policy:

```bash
uv run src/run.py --policy unitree_12dof --prefix g1
uv run src/run.py --ui                          # picks them up in the policy tab
```

To use the registry from code:

```python
from urlab_policy.registry import POLICIES, get_policy_labels

print(get_policy_labels())
print(POLICIES["unitree_12dof"]["dofs"])
```

## Bundled policies

| Key | Robot | DOF | MJCF | Notes |
|---|---|---|---|---|
| `unitree_12dof` | G1 | 12 | `urlab_bridge/assets/robots/g1/g1_29dof_rev_1_0.xml` | Twist control, locomotion |
| `unitree_wo_gait` | G1 | 29 | `urlab_bridge/assets/robots/g1/g1_29dof_rev_1_0.xml` | Full body, no gait clock |
| `smooth` | G1 | 29 | `urlab_bridge/assets/robots/g1/g1_29dof_rev_1_0.xml` | Smoother locomotion |
| `beyondmimic_dance` | G1 | 29 | `urlab_bridge/assets/robots/g1/g1_29dof_rev_1_0.xml` | Motion imitation. Requires PHC |
| `h2h` | G1 | 21 | `urlab_bridge/assets/robots/g1/g1_29dof_rev_1_0.xml` | Human motion retargeting. Requires PHC |
| `amo` | G1 | 29 | `urlab_bridge/assets/robots/g1/g1_29dof_rev_1_0.xml` | Adaptive motion optimisation. Requires PHC |
| `twist_tracker` | G1 | 12 | `urlab_bridge/assets/robots/g1/g1_29dof_rev_1_0.xml` | Motion tracking with twist. Requires PHC |
| `go2_wtw` | Go2 | 12 | `mujoco_menagerie/unitree_go2/go2.xml` (download) | Walk-These-Ways quadruped |

The G1 XMLs ship in `urlab_bridge/assets/robots/g1/` with their mesh
dirs. Go2 must be grabbed from
[mujoco_menagerie](https://github.com/google-deepmind/mujoco_menagerie).
Whichever XML you import, the resulting Articulation Blueprint needs a
`UMjPDController` added before any of these policies will move it. See
[Running a Bundled Policy](running_policies.md) for the full setup.

PHC-flagged entries need
[PHC](https://github.com/ZhengyiLuo/PHC) installed inside the RoboJuDo
submodule.

## Adapters

The merged [`urlab_policy.registry.POLICIES`](https://github.com/URLab-Sim/urlab_bridge/blob/main/src/urlab_policy/registry.py)
dict is the union of every adapter's `registry.py`. Two adapters
ship today:

- [`adapters/robojudo/`](https://github.com/URLab-Sim/urlab_bridge/tree/main/src/urlab_policy/adapters/robojudo) —
  the RoboJuDo policy ecosystem. All eight bundled entries above live
  in [`adapters/robojudo/registry.py`](https://github.com/URLab-Sim/urlab_bridge/blob/main/src/urlab_policy/adapters/robojudo/registry.py).
- [`adapters/mjlab/`](https://github.com/URLab-Sim/urlab_bridge/tree/main/src/urlab_policy/adapters/mjlab) —
  mjlab-trained policies, run via the framework-agnostic `PolicyRunner` +
  `TaskSpec` shape.

## Adding a new policy

Drop an entry into the relevant adapter's `registry.py` — copy an
existing one as a template. The schema is whatever fields the adapter
reads; the easiest way to see what's required is to open the adapter
source. The policy and config classes go wherever the adapter expects
(`urlab_policy/policies/` for URLab-bundled, inside the framework
itself for RoboJuDo / mjlab natives).

After the entry lands:

```bash
uv run urlab-policy --policy my_policy --prefix robot
uv run urlab-ui                            # picks it up in the policy tab
```

The dashboard imports the merged registry on launch, so new entries
show up next session.

## `required_step_mode`

Mode-sensitive policies declare what they need. The launcher refuses
to start a policy in an incompatible mode rather than letting you find
out via silent garbage rollouts.

```python
"my_policy": {
    "label": "...",
    # ...
    "required_step_mode": "direct",                 # single mode
    # or
    "required_step_mode": ("direct", "puppet"),     # any-of
}
```

| Policy traits | Declare what |
|---|---|
| Calls `mj_step` on `client.data` itself (MJX, custom integrator) | `puppet` |
| Reads `art.controller` PD gains, expects UE to step | `direct` |
| Wraps a teleop input device, needs continuous publishing | `live` |
| Pure black-box `act(obs) -> action` | omit (works in all modes) |

The check is `urlab_policy.registry.check_step_mode_compatible`,
called from `run.py` and the UI launcher before opening a session.

## Skipping the registry

The registry is for pretrained policies that should be discoverable
from the dashboard. If you're writing a one-off control loop, skip it
entirely and use `URLabClient` directly:

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

See the bridge's
[Getting Started](https://github.com/URLab-Sim/urlab_bridge/blob/main/docs/getting_started.md#add-a-new-policy)
for the full custom-loop walkthrough.

## Cross-references

- [Networking](../guides/networking.md): the three step modes themselves.
- [Step Server Protocol](../reference/step_server.md): wire-level reference.
- [URLab Bridge](bridge.md): bridge install + dashboard launcher.
