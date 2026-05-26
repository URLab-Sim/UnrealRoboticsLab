# Gym Environment

`URLabEnv` is a thin `gymnasium.Env` wrapper around a single
`URLabClient`. Use it when you have a policy trained elsewhere and
want it to step through URLab's contact physics + rendering, or when
you need a `gym.Env` interface for a script that already speaks one.

> **Not designed for high-throughput training.** It's one process,
> one editor, one env. Vectorised GPU sims like mjlab, MJX, and
> `mujoco_warp` train policies orders of magnitude faster. Reach for
> those for the inner loop and use `URLabEnv` for the rollout that
> matters — eval, sim-real verification, or a sanity rollout against
> URLab's higher-fidelity contacts and rendering.

## When `URLabEnv` is the right shape

- **Evaluation.** Train on a fast sim; evaluate against URLab's
  rendering / contacts / scene authoring. One env is fine for eval.
- **Smoke-testing a training pipeline.** Verify your SB3 / CleanRL /
  custom-trainer plumbing handles a real `gym.Env` before pointing
  it at the production fast sim.
- **Single-env online loops.** Teleop-in-the-loop, sim-real bridging,
  n=1 deterministic rollouts, scripted benchmarking.

For everything else — driving the sim, scene authoring, sending
control, recording episodes — use `URLabClient` directly; see
[Python Getting Started](getting_started.md). For the underlying
transport see [Networking](../guides/networking.md).

## Minimal usage

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

`URLabEnv` wraps an already-discovered `URLabClient` (the constructor
takes the client as its first positional argument). The compiled
`MjModel` is loaded as `env.client.model` and the underlying client
is reachable as `env.client` for anything the gym surface does not
expose.

## `space_mode="flat"` vs `"dict"`

Pick at construction:

```python
URLabEnv(client, space_mode="flat")   # default
URLabEnv(client, space_mode="dict")
```

| Mode | Action / observation type | Use when |
|---|---|---|
| `flat` | `Box` (concatenated qpos + qvel + sensor values across articulations in discovery order) | Single-articulation rollouts, anything that expects a flat actor-critic input |
| `dict` | `Dict` keyed by articulation prefix | Multi-articulation scenes where you want each prefix addressable, or observation-structure-aware nets |

The underlying `URLabClient` is the same in both modes; only the
space wrapper changes.

## `reward_fn` and `termination_fn`

Both opt-in callables. URLab does not invent rewards for you; if you
do not provide one, `reward` is `0.0` and `terminated` is `False` for
every step.

Both callables take the same `info` dict the env returns from
`step()` (single positional arg). The dict carries the live
`URLabClient`, the per-articulation collections, the `step_count`,
and `sim_time`.

```python
def my_reward(info) -> float:
    art = info["client"].articulations["vx300s"]
    waist_qpos = art.qpos_array[art.joints["waist"].qpos_local_offset]
    return -abs(waist_qpos - 0.5)

def my_termination(info) -> bool:
    art = info["client"].articulations["vx300s"]
    waist_qpos = art.qpos_array[art.joints["waist"].qpos_local_offset]
    return waist_qpos > 1.5

env = URLabEnv(
    client,
    reward_fn=my_reward,
    termination_fn=my_termination,
)
```

The `Joint` dataclass exposes layout (`qpos_offset`,
`qpos_local_offset`, `qpos_dim`, `range`, …) but not the live
position. To read joint state, index into the articulation's flat
buffer with the joint's local offset (above), or call
`art.get_qpos()` for a `{name: float}` dict (first qpos component per
joint).

If you need richer state (contact forces, body `xpos`), construct
the client with `observations="full"` so the wire reply carries that
data and the wrappers populate it.

## `seed()` semantics

`reset(seed=...)` writes the seed onto the reset RPC, which sets
`m->opt.seed` server-side. The same seed plus the same scene plus
the same starting keyframe gives a bit-reproducible rollout in
`direct` mode.

```python
obs, info = env.reset(seed=42)
```

Subsequent `reset()` calls without a seed reuse the most recent one.
To rotate seeds per episode, pass it explicitly each time.

In `puppet` mode the client owns the integrator, so reproducibility
is governed by the client's MuJoCo install (already bit-deterministic
at the same `mujoco==` pin). Server-side seeding still applies to
anything UE samples (random reset offsets, sensor noise).

## `max_episode_steps` truncation

```python
env = URLabEnv(client, max_episode_steps=1000)
```

After 1000 `env.step()` calls without termination, the env returns
`truncated=True` and `terminated=False`. The next `reset()` clears
the counter. This is the standard gym truncation contract.

If you do not pass `max_episode_steps`, episodes only end on
`terminated=True` from your `termination_fn`.

## Observation level

```python
URLabEnv(client, observations="standard")   # default
URLabEnv(client, observations="minimal")
URLabEnv(client, observations="full")
```

Trades wire bandwidth for observation richness. See the
[Step Server reference](../reference/step_server.md#observation-verbosity)
for the per-level contents.

## Switching to puppet at the env level

```python
from urlab_client import URLabClient
from urlab_policy.adapters.robojudo.env import URLabEnv

client = URLabClient("tcp://localhost", step_mode="puppet")    # client owns mj_step
client.discover()
env = URLabEnv(client)
obs, info = env.reset()
```

In puppet, the env still satisfies the gym contract; under the hood
it pushes the locally integrated `client.data` to UE for rendering
each step. `action` semantics are unchanged from the policy's
perspective. Pick puppet when your trainer is MJX, `mujoco_warp`, or
anything else that owns its own integrator.

`reward_fn` and `termination_fn` work identically in all modes.

## Closing cleanly

```python
env.close()
```

Tears down the underlying `URLabClient` socket. Always call it; UE
keeps the session alive (and pauses publishers in stepped modes)
until the client disconnects.

