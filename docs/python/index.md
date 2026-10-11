# Python and external control

The companion [urlab_bridge](https://github.com/URLab-Sim/urlab_bridge) repository provides the current typed Python clients. Install it with `uv sync` (Python 3.11 or 3.12); add `uv sync --extra grpc` only for the Linux gRPC render extension. The [bridge README](https://github.com/URLab-Sim/urlab_bridge#readme) is the Python entry point.

Create a `Connection` to an already running service and call `connect()` explicitly. Connecting is passive. Compose the client for the service advertised by that connection:

| Task | Client and authority |
|---|---|
| Step, observe, reset, or checkpoint a URLab simulation | `SimulationClient`; explicitly claim simulation control |
| Install a scene and request exact RGB, depth, or segmentation captures | `RenderClient` with `DataClient`; explicitly attach a render owner |
| Install a scene or bind a pose producer to a visible viewer | `ViewerClient`; explicitly attach a viewer owner |
| Author a level in the editor | `EditorClient`; editor service only |
| Publish Cartesian poses or read advertised pose/camera output | `PoseStreamPublisher` or passive `OutputReader` |
| Distribute captures among independent render workers | `RenderPool`; connect, attach owners, then install the source |

The [typed render guide](https://github.com/URLab-Sim/urlab_bridge/blob/main/docs/render_server.md) shows an owner, source install, capture and result release. The [pool guide](https://github.com/URLab-Sim/urlab_bridge/blob/main/docs/render_pool.md) shows multiple workers. Runnable bridge examples include `examples/render_client_example.py`, `examples/render_server_viewer.py` (an OpenCV display of render-server pixels), and `examples/owner_viewer_bunny.py` (a visible Unreal viewer).

## Data and transport

Use `tcp://host:port` for ZMQ commands. The connection negotiates network or shared-memory data; shared memory is **not** a command endpoint. Exact uploads and captures use retained payload descriptors. Live pose, state and camera output uses separately advertised latest-frame streams, so readers must use the returned descriptor and reopen after a generation or revision change. The optional `grpc://` endpoint carries URLab render extensions through `dm_env_rpc` on Linux with network data. It does not provide simulation control or a continuous viewer bus. See the [transport contract](../architecture/transport_design.md).

The earlier `URLabClient`, runtime articulation wrappers, implicit PIE/control behavior, SHM RPC commands and recording/replay calls were removed. There are no compatibility aliases. For the service boundaries and verified scope, see [intended design](../architecture/intended_design.md) and [completion status](../architecture/completion_status.md).
