# Driving URLab from Python

URLab ships with a typed Python client (`URLabClient`, in the
[urlab_bridge](https://github.com/URLab-Sim/urlab_bridge) companion
repository) that gives you full programmatic control of an Unreal
session: scene authoring, PIE lifecycle, sending control, reading
state, capturing cameras, and recording / replaying episodes.

**Start with [Getting Started](getting_started.md)** — it walks the
common flows end-to-end. Then keep [API Reference](api.md) open as
you build.

## Start here

| Doc | When to read it |
|-----|-----------------|
| [Getting Started](getting_started.md) | First read. Connect, author a scene, run PIE, send control, tune gains, work with cameras, add a policy. |
| [Running a Bundled Policy](running_policies.md) | Step-by-step from a fresh install to a policy driving the robot. Pick this if you just want to see something move. |
| [API Reference](api.md) | Every method on `URLabClient` and the `scene` / `sim` / `runtime` / `outliner` / `debug` / `viewport` / `recording` / `replay` namespaces, with reply-field contracts. |

## Topic guides

### Bridge install + dashboard

- [Bridge install + dashboard](bridge.md) — set up the
  `urlab_bridge` companion repo, launch the DearPyGui dashboard, run
  bundled pretrained policies, and see how the legacy ZMQ streaming
  protocol fits in.

### Environment integration

- [Gym Environment](gym_environment.md) — `URLabEnv(gym.Env)` wrapper
  for users integrating URLab with their own training pipelines
  (stable-baselines3, CleanRL, RSL-RL, etc.).
- [Policy Registry](policy_registry.md) — bundled pretrained policies
  (Unitree G1, Go2 Walk-These-Ways, BeyondMimic, etc.) and how to add
  your own to the registry.

### Control and runtime

- [Controllers and Gains](controllers_and_gains.md) — `ue_controller`
  vs `raw` modes, `n_steps` decimation, runtime PD gain updates.
- [Entities](entities.md) — non-articulation dynamic objects
  (free-jointed props, kinematic bodies) under `client.entities`.

### Recording

- [Recording and Replay](recording_replay.md) — capture episodes as
  `qpos` / `qvel` snapshots, save to disk, play back deterministically
  through the same custom-step-handler primitive.

## Related (UE-side and wire protocol)

- [Networking](../guides/networking.md) — UE-side architecture for
  the three step modes (`live`, `direct`, `puppet`), the two transports
  (ZMQ, shared memory), the legacy PUB/SUB wire format, and the ROS 2
  bridge.
- [Step Server Protocol](../reference/step_server.md) — wire-protocol
  reference for the REP socket on port 5559.
