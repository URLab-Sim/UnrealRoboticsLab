# gRPC / dm_env_rpc transport setup (Linux)

The `URLabDmEnvRpc` module gives the bridge a gRPC backend that speaks the
DeepMind `dm_env_rpc` proto, so a company client can drive a render server over
gRPC. It is **Linux-only** (`PlatformAllowList: ["Linux"]` in the `.uplugin`) and
links a locally-built static gRPC — the Windows editor skips it entirely.

It is a normal transport-role backend: at `StartupModule` it binds
`FMjExternalTransportProvider::MakeControlRpcTransport`, so the bridge creates a
`UURLabDmEnvRpcTransport` (a `UURLabRpcTransport`) instead of ZMQ. Requests arrive
as a `UrlabPacket` in the proto's `extension` field, are handed to
`ProcessRequestBytes` (the same dispatch every transport uses), and the reply is
packed back into `extension`. So every bridge op — including `fastpath_render` —
works over gRPC unchanged.

## 1. Build gRPC (one time, per Linux box)

Prereqs: `git`, `cmake` (>= 3.16), a C++17 toolchain (clang/gcc), `make`.

```bash
cd Plugins/UnrealRoboticsLab/third_party/grpc
./build.sh            # clones grpc v1.62.0 into ./src, cmake-builds static libs,
                      # installs to ../install/grpc  (include/ + lib/*.a)
```

This is a large build (~15-40 min). The install lands at
`third_party/install/grpc`, which `URLabDmEnvRpc.Build.cs` looks for (`include/`
and every `lib/*.a`). No system install — everything stays under the plugin.

## 2. Build the plugin on Linux

With `third_party/install/grpc` present, a normal Linux `url_projEditor` (or a
packaged Linux target) build compiles `URLabDmEnvRpc` and links the static gRPC.
If the link reports undefined Windows-vs-Unix symbols, adjust the platform system
libraries in `URLabDmEnvRpc.Build.cs` (the Linux list is `pthread dl rt z`).

## 3. Wire status

- **Extension tunnel — ready.** Every op (control + `fastpath_render`) tunnels our
  msgpack through `extension`, unpacked by us. This is the mandated wire.
- **Native `step` — next phase.** The transform-update loop will map to native
  `dm_env_rpc` `StepRequest.actions` (transforms as float Tensors) /
  `StepResponse.observations` (camera images as uint8 Tensors), so a stock dm_env
  client can drive it; everything else stays on the extension tunnel. The proto
  `step` handlers are currently stubs — that mapping is the remaining work.

## Notes

- The gRPC pin is v1.62.0 (see `build.sh`). Generated proto C++ is checked in under
  `Source/URLabDmEnvRpc/Private/GenProto`, so no protoc run is needed to build.
- To disable the backend, remove the module from the `.uplugin` (or leave
  `third_party/install/grpc` absent — the Build.cs adds no grpc libs and the module
  won't link, so keep it registered only once gRPC is built).
