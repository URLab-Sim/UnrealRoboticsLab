# URLab transport design

Transport moves commands, exact payloads, and live streams; it does not select the physics clock or grant scene/control authority. These are the intended profiles:

| Profile | Commands | Exact and live data |
|---|---|---|
| Local | ZMQ | Negotiated shared-memory data for substantial payloads |
| Remote | ZMQ | Network payload transfer and streams |
| gRPC render extension | `URLabDmEnvRpc` / `Environment.Process` carrying URLab packets | Network payload transfer for rendering |

`tcp://` identifies a ZMQ command endpoint. SHM is a negotiated data path, not an RPC listener. A required SHM request fails clearly when unavailable; `auto` reports its selected path. Once selected, a broken mapping is an error that requires renegotiation rather than a hidden per-operation fallback. gRPC does not bootstrap through ZMQ or choose SHM. Native dm_env operations, simulation stepping, editor authoring and viewer subscriptions are outside the gRPC extension scope.

A passive connection first validates protocol and receives instance/boot identity, available services and operations, limits, and data offers. SHM negotiation proves access to a server-created challenge object, rather than trusting a hostname or path. Workflow attachment then establishes its own owner or control identity. A restart changes boot identity even when an endpoint is reused. Readers do not gain write authority, and reconnect does not silently reattach an owner.

Commands use typed, correlated request envelopes with protocol version, session, request ID and operation. Mutations include the relevant authority and expected revision. A bounded per-session ledger retains accepted IDs and payload digests: a repeat of the same ID/payload returns the retained status/result; a conflicting repeat is rejected. After a timeout, a caller may recover using that same logical ID and session if the same boot is alive. An unknown outcome is never treated as permission to execute again under a new ID. Long GPU and editor work admits promptly and returns a job handle.

Exact scene uploads, state snapshots and capture results use retained immutable payloads. Descriptors bind bytes to boot/session identity, size, digest, expiry and transfer method; domain wrappers add revision, shape, encoding and units. Local exact SHM objects are sealed before publication, copied under validated lifetime, and released or expired explicitly. Network payloads use bounded chunks with offsets and integrity checks. An accepted render job reserves result capacity; unread results survive until collection/release or an advertised expiry, never silently turn into another frame. Camera history has a separate retention budget.

Live pose, observation and camera streams are latest-frame channels. They may skip intermediate frames but each returned frame must have a coherent header/payload. Descriptors carry kind/version, stream and producer/boot identity, revision, sequence, source time and clock epoch. Camera frames additionally specify camera identity/revision, dimensions, stride and pixel format. Unknown versions are errors. Local SHM streams use interprocess locking around copies; producers enqueue immutable snapshots and never wait for readers on a physics or GPU thread. ZMQ stream endpoints come from discovery, not reconstructed port arithmetic.

A render pool shares the same connection negotiation as one client while assigning cameras among independent workers. Process reservation, source ownership, and result identity remain separate. The bridge collects only results matching the submitted scene and frame. ROS is a later, separate simulation input/output and camera publication adapter, not another transport profile for the renderer.

This page states the transport contract. [Completion status](completion_status.md) records current verification and platform limits.
