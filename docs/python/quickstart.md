# Python quickstart

Start with the [current Python overview](index.md) and the companion bridge's [typed render guide](https://github.com/URLab-Sim/urlab_bridge/blob/main/docs/render_server.md). Its runnable `examples/render_client_example.py` connects to an existing render service, attaches an owner, installs XML, submits `qpos` captures and releases their payloads. For a visible Unreal viewer driven by Python-owned MuJoCo physics, see `examples/owner_viewer_bunny.py` in the same repository.

The old `URLabClient`/articulation and `client.sim.start()` walkthrough has been retired. `Connection` is passive; simulation control, render ownership and viewer ownership are separate explicit claims. See [service scope and transports](index.md) before adapting an older script.
