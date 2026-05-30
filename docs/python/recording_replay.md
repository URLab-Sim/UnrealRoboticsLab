# Recording and Replay

Every physics step through URLab can be captured as a per-joint `qpos` / `qvel` frame keyed by sim-time, saved as JSON, and played back deterministically. The recorder hooks `UMjPhysicsEngine::OnPostStep`, so recording works identically in `live`, `direct`, and `puppet` modes. Playback reuses the same custom-step-handler primitive the physics engine exposes.

All commands below assume a connected `URLabClient`; see the [Networking guide](../guides/networking.md) for setup.

## Recording an episode

```python
from urlab_client import URLabClient, StepMode

client = URLabClient(step_mode=StepMode.DIRECT)
client.discover()

client.recording.start(name="ep_1234")

art = client.articulations["vx300s"]
for _ in range(1000):
    art.set_ctrl({"waist": 0.5})
    client.step(n_steps=5)

client.recording.stop()
path = client.recording.save("ep_1234.json")
print(path)   # absolute path actually written
```

`recording.start(name=None)` auto-generates a name from timestamp plus step count if you omit one. Call it at any point; the recorder starts capturing on the next `OnPostStep`.

Useful attributes while a recording is active:

```python
client.recording.is_active       # bool
client.recording.frame_count     # frames captured so far
client.recording.sim_duration    # seconds of sim time captured
client.recording.last_saved_path # absolute path or None
```

`recording.stop()` freezes the capture; frames stay in memory until you call `save()` or `clear()`.

## Default duration is effectively infinite

`AMjReplayManager.MaxRecordDuration` defaults to 60 seconds when the recorder is driven from the editor Details panel. That limit is too short for training-length episodes, so `recording.start` overrides it to `FLT_MAX` (~`3.4e38` seconds) unless you pass an explicit limit:

```python
client.recording.start(name="short_demo", max_duration_s=30.0)
```

The editor's 60-second default stays as-is for users who record from the Details panel.

## Playing one back

One-shot convenience: load the file, set active, start playback.

```python
client.replay.play("ep_1234.json")
```

That resolves `path_or_name` against loaded sessions (by name) and the filesystem (by path), loads if needed, and starts. `loop=True` restarts from frame 0 when the session ends.

The explicit three-call form is also supported:

```python
session = client.replay.load("ep_1234.json")   # ReplaySession
client.replay.set_active(session.name)
client.replay.start()
```

While replay is active, `client.step(n)` advances `n` frames of the recorded timeline. `ctrl` and `xfrc_applied` on the step request are ignored; observations come from the replayed `qpos` after `mj_forward` runs. The reply adds `"replay": {"frame": N, "total_frames": M}`.

```python
client.replay.list_sessions()   # ["live", "ep_1234", ...]
client.replay.active_session    # "ep_1234"
client.replay.stop()
```

After `replay.stop()`, the next `client.step(n)` issues live `mj_step` calls again. `sim_time` continues forward from wherever the replay left off; call `client.reset()` if you want a clean slate.

`replay.start()` requires the server to be in `direct` or `puppet` (client owns the step cadence). Calling it in `live` returns `error(code="replay_requires_stepped")`.

## Save path resolution

Save paths follow three rules, checked in order:

1. **Bare filename** (`"ep_1234.json"`): resolved under `<Project>/Saved/URLab/Replays/`.
2. **Absolute path** (`"C:/logs/ep_1234.json"`): written verbatim.
3. **Reply always contains the absolute path.** `recording.save` returns it; `client.recording.last_saved_path` tracks it.

`replay.load` uses the same resolution.

Example output from the save reply:

```
C:/Users/you/Documents/Unreal Projects/MyProj/Saved/URLab/Replays/ep_1234.json
```

## Recording across all three modes

Because `OnPostStep` fires exactly once per `mj_step` regardless of what drove the step, the recorder does not care which mode you are in:

- **`live`**: every sim-thread step produces a frame. Good for capturing teleop sessions or ROS-driven demos.
- **`direct`**: every `step(n)` produces `n` frames. Good for capturing scripted rollouts and policy demos.
- **`puppet`**: the server fires `OnPostStep` manually after `mj_forward`, so each `step(n)` produces exactly one frame (puppet does not sub-step on the UE side). Good for capturing MJX or Jax-owned rollouts.

Sim time is the authoritative axis in every case. A recording captured at 10x realtime in `direct` replays at wall-clock speed with no tempo jitter.

## Use case: capture many episodes, cinematic-replay the best

```python
for episode in range(10_000):
    client.recording.start(name=f"ep_{episode}")
    run_policy(client)
    client.recording.stop()
    client.recording.save(f"ep_{episode}.json")
    if keep_episode(episode):
        print(f"keep {episode}")
    else:
        client.recording.clear()
```

Re-open the saved file in URLab (or via `client.replay.play(...)`) and play back at wall-clock speed with cameras, lighting, and post effects live. No screen capture, no tempo jitter.

## Error codes

| Code | Meaning |
|---|---|
| `recording_already_active` | `start` issued while another recording is running. |
| `recording_not_active` | `stop` / `save` issued without an active recording. |
| `path_not_writable` | `recording_save` target path is not writable. |
| `path_not_readable` | `replay_load` source path is not readable. |
| `replay_requires_stepped` | `replay_start` issued in `live`. |
| `replay_session_not_found` | `set_active` / `start` named a session that is not loaded. |

See the [Step Server wire protocol](../reference/step_server.md#recording) for the raw message shapes.
