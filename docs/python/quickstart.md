# Python quickstart

Start with the [Python overview](index.md) and the companion bridge's [Getting Started](https://github.com/URLab-Sim/URLab_Bridge/blob/main/docs/getting_started.md) and [Typed Render Service](https://github.com/URLab-Sim/URLab_Bridge/blob/main/docs/render_server.md) guides.

- **Headless render server:** `examples/render_client_example.py` (or `examples/quickstart.py` for the drop-in `urlab_client.Renderer`) connects to a render service, attaches an owner, installs XML, submits `qpos` captures, and releases payloads.
- **Visible Unreal viewer:** `examples/owner_viewer_bunny.py` drives a visible Unreal viewer from Python-owned MuJoCo physics.

`Connection` is passive; simulation control, render ownership, and viewer ownership are separate explicit claims.
