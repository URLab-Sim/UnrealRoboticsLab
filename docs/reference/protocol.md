# Protocol reference

The former operation catalogue described the retired universal client, SHM command ring, runtime articulation controls and recording/replay operations. It is not a reference for the current protocol.

The source of truth for current operations and request shapes is `wire/urlab_wire.yaml` in the plugin repository, with generated counterparts in the companion bridge's `wire/` directory. Read the [transport contract](../architecture/transport_design.md) for session identity, ZMQ commands, negotiated exact data, live streams and result retention; read the [intended design](../architecture/intended_design.md) for service authority. The [typed render guide](https://github.com/URLab-Sim/urlab_bridge/blob/main/docs/render_server.md) shows the client-facing workflow.
