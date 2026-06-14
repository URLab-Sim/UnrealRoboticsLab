# Recording & Replay

Capture every physics step as a per-joint `qpos`/`qvel` frame keyed by sim-time, save it as a `.json` file, and play it back deterministically. Recording works the same from the editor and from Python.

Because the recorder hooks the physics engine's post-step event, it captures one frame per `mj_step` regardless of what drove the step, so it behaves identically across all step modes. Sim time is the authoritative axis: a session captured at 10x real time in a scripted rollout replays at wall-clock speed with no tempo jitter.

![The dashboard recording and replay controls with a saved session selected](../images/placeholder.svg)

## From the editor

Use the [Simulate Dashboard](dashboard.md):

- **Record**: start and stop capturing the running simulation.
- **Save**: write the captured frames to a `.json` file.
- **Replay**: load a saved session and play it back, with cameras, lighting, and post effects live.

!!! note
    When driven from the editor Details panel, the recorder's `MaxRecordDuration` defaults to 60 seconds. That is fine for short demos but too short for long evaluation episodes. Python recording overrides this to effectively unlimited (see below).

## From Python

All commands below assume a connected client. See [Python Quickstart](../python/quickstart.md).

### Record an episode

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

`recording.start(name=None)` auto-generates a name from the timestamp and step count if you omit one, and begins capturing on the next post-step. While a recording is active you can read `client.recording.is_active`, `.frame_count`, `.sim_duration`, and `.last_saved_path`. `recording.stop()` freezes the capture; frames stay in memory until you `save()` or `clear()`.

Python recording overrides the editor's 60-second cap to effectively infinite. Pass an explicit limit to keep one:

```python
client.recording.start(name="short_demo", max_duration_s=30.0)
```

### Play one back

```python
client.replay.play("ep_1234.json")   # load, set active, start
```

The explicit form is also available:

```python
session = client.replay.load("ep_1234.json")
client.replay.set_active(session.name)
client.replay.start()
```

While replay is active, `client.step(n)` advances `n` frames of the recorded timeline; `ctrl` and `xfrc_applied` on the step request are ignored, and observations come from the replayed `qpos`. The reply adds a `replay` block with the current and total frame counts. Use `client.replay.list_sessions()`, `.active_session`, and `.stop()` to manage playback. After `stop()`, the next step issues live steps again; call `client.reset()` for a clean slate.

!!! warning
    `replay.start()` requires the server to be in `direct` or `puppet` mode, so the client owns the step cadence. Calling it in `live` mode returns `error(code="replay_requires_stepped")`.

### Save path resolution

- A **bare filename** (`"ep_1234.json"`) resolves under `<Project>/Saved/URLab/Replays/`.
- An **absolute path** (`"C:/logs/ep_1234.json"`) is written verbatim.
- The save reply always contains the absolute path actually written, also tracked in `client.recording.last_saved_path`. `replay.load` uses the same resolution.

## Capture many, replay the best

A common loop: record every episode, keep the good ones, and clear the rest.

```python
for episode in range(10_000):
    client.recording.start(name=f"ep_{episode}")
    run_policy(client)
    client.recording.stop()
    client.recording.save(f"ep_{episode}.json")
    if not keep_episode(episode):
        client.recording.clear()
```

Re-open a kept file in the editor or via `client.replay.play(...)` and play it back at wall-clock speed with full rendering. No screen capture, no tempo jitter.

## Next steps

- [Python Quickstart](../python/quickstart.md) for connecting a client.
- [Simulate Dashboard](dashboard.md) for the editor record and replay controls.
- [Step server protocol](../reference/protocol.md) for the raw recording and replay message shapes.
