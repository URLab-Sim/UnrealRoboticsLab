# Python API reference

The previous `URLabClient` namespace reference described an API that has been removed. Current Python code uses `Connection`, `SimulationClient`, `RenderClient`, `ViewerClient`, `EditorClient`, `PoseStreamPublisher`, `OutputReader` and `RenderPool` from [urlab_bridge](https://github.com/URLab-Sim/urlab_bridge). These are separate service clients and require explicit connection and authority.

See the [Python overview](index.md), [typed render guide](https://github.com/URLab-Sim/urlab_bridge/blob/main/docs/render_server.md), [pool guide](https://github.com/URLab-Sim/urlab_bridge/blob/main/docs/render_pool.md), and bridge `src/urlab_client/` for current method signatures. The [intended design](../architecture/intended_design.md) describes the boundary of each service.
