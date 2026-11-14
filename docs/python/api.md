# Python API reference

The previous `URLabClient` namespace reference described an API that has been removed. Current Python code uses `Connection`, `SimulationClient`, `RenderClient`, `ViewerClient`, `EditorClient`, `PoseStreamPublisher`, `OutputReader` and `RenderPool` from [urlab_bridge](https://github.com/URLab-Sim/URLab_Bridge). These are separate service clients and require explicit connection and authority.

See the [Python overview](index.md), typed render guide (`docs/render_server.md` in [URLab_Bridge](https://github.com/URLab-Sim/URLab_Bridge)), pool guide (`docs/render_pool.md` in [URLab_Bridge](https://github.com/URLab-Sim/URLab_Bridge)), and bridge `src/urlab_client/` for current method signatures. The [intended design](../architecture/intended_design.md) describes the boundary of each service.
