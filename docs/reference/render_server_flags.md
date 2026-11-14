# Workflow launch flags

The supported workflow selector is `-URLabWorkflow=simulation|render|viewer`. The parser rejects the retired `-URLabDrive` and `-URLabSourceFind` flags. Outside the editor, a refused launch or a failed bind exits the process with status 3. The authoritative grammar is in `Source/URLab/Public/MuJoCo/Render/MjLauncherFlags.h`.

| Flag | Current use |
|---|---|
| `-URLabWorkflow=simulation` | Start the world-owned simulation service. |
| `-URLabWorkflow=render` | Start a passive render service; a client attaches an owner and installs a source. |
| `-URLabWorkflow=viewer` | Start a visible viewer; a client attaches an owner and installs or binds a source. |
| `-URLabSimulationSource=<manifest.json>` | Simulation only. Manifest: `{"format":"mjb"|"mjz"|"xml","path":"...","assets":{"vfsName":"filePath"}}`; assets are optional. |
| `-URLabClock=stepped|freerun` | Simulation clock selection only. |
| `-URLabCaps=<csv>` | Service, output and permission capabilities from the generated wire vocabulary. |
| `-URLabScene=<csv>` | Scene options. The render workflow applies only `cammax=<px>` (`0` keeps the authored resolution) and refuses other keys; the viewer workflow refuses all keys. |
| `-URLabNet=<csv>` | Network settings: `id=`, `index=`, `step=`, `grpc=`, `bind=` (default `127.0.0.1`; `0.0.0.0` serves every interface without authentication) and `parent=<pid>` (exit when that process is gone). |

For a packaged render service, pass `-URLabWorkflow=render` to its staged launcher with a suitable lit map and `-RenderOffScreen`; then use the bridge's typed render guide (`docs/render_server.md` in [URLab_Bridge](https://github.com/URLab-Sim/URLab_Bridge)) to attach an owner and install a source. gRPC is a Linux-only URLab render extension over `dm_env_rpc`; use a `grpc://` endpoint and network data. Simulation commands and continuous viewer subscriptions use other paths. See [render roles](../concepts/render_roles.md), [packaging](../guides/render_server_packaging.md), and the [transport contract](../architecture/transport_design.md).
