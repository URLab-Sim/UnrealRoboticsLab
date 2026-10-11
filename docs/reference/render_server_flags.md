# Workflow launch flags

The supported workflow selector is `-URLabWorkflow=simulation|render|viewer`. The parser rejects the retired `-URLabDrive` and `-URLabSourceFind` flags. The authoritative grammar is in `Source/URLab/Public/MuJoCo/Render/MjLauncherFlags.h`.

| Flag | Current use |
|---|---|
| `-URLabWorkflow=simulation` | Start the world-owned simulation service. |
| `-URLabWorkflow=render` | Start a passive render service; a client attaches an owner and installs a source. |
| `-URLabWorkflow=viewer` | Start a visible viewer; a client attaches an owner and installs or binds a source. |
| `-URLabSimulationSource=<manifest.json>` | Simulation only. Manifest: `{"format":"mjb"|"mjz"|"xml","path":"...","assets":{"vfsName":"filePath"}}`; assets are optional. |
| `-URLabClock=stepped|freerun` | Simulation clock selection only. |
| `-URLabCaps=<csv>` | Service, output and permission capabilities from the generated wire vocabulary. |
| `-URLabScene=<csv>` | Scene options such as `cammax=0` for source camera resolution. |
| `-URLabNet=<csv>` | Network settings, including `step=`, `grpc=`, `index=` and `bind=`. |

For a packaged render service, pass `-URLabWorkflow=render` to its staged launcher with a suitable lit map and `-RenderOffScreen`; then use the bridge's [typed render guide](https://github.com/URLab-Sim/urlab_bridge/blob/main/docs/render_server.md) to attach an owner and install a source. gRPC is a Linux-only URLab render extension over `dm_env_rpc`; use a `grpc://` endpoint and network data. Simulation commands and continuous viewer subscriptions use other paths. See [render roles](../concepts/render_roles.md), [packaging](../guides/render_server_packaging.md), and the [transport contract](../architecture/transport_design.md).
