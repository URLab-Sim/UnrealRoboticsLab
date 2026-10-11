# Networking

URLab uses explicit service commands and separate data delivery. A `Connection` establishes a passive session and receives the available services, permissions and data offers. The simulation controller, render owner and viewer owner then make separate claims.

| Path | Purpose |
|---|---|
| ZMQ `tcp://` | Typed commands, including simulation and editor operations; network data when selected. |
| Negotiated shared memory | Same-host exact payloads and advertised latest-frame streams. There is no SHM RPC command endpoint. |
| gRPC `grpc://` | Linux-only URLab render extensions inside `dm_env_rpc`, with network data. No general simulation or viewer command support. |

Exact uploads and capture results are retained immutable payloads with identity, lifetime and release. Live state, pose and camera streams are latest-frame channels and may skip intermediate frames; readers use the advertised descriptor and reopen after a generation or revision change. See the [transport design](../architecture/transport_design.md) for session, retry and data contracts, and [Python overview](../python/index.md) for the clients.
