# The gRPC (dm_env_rpc) transport

`URLabDmEnvRpc` gives the bridge a gRPC backend that speaks DeepMind's
`dm_env_rpc` proto, so a client can issue URLab render requests over gRPC. It does
not provide simulation control or the continuous viewer/owner subscribe bus.
It is **Linux-only**: the `.uplugin` carries `PlatformAllowList: ["Linux"]`,
so on any other platform the module is skipped at build time and nothing registers
the `grpc://` scheme.

## How it plugs in

The module is a transport backend, not a replacement for the bridge. At startup it
appends its factory to the bridge's control-RPC transport factory list, so the ZMQ
transport still binds and gRPC binds alongside it. A request arrives as a
`UrlabPacket` in the proto's `extension` field, is handed to the same
`ProcessRequestBytes` dispatch used by supported URLab extension operations, and the
reply is packed back into `extension`. `render_frame` is the required gRPC workflow.

The listener binds `:50051` by default. Move it with `-URLabNet=grpc=<port>`.

Because the module is absent on Windows and macOS, a `grpc://` endpoint there
resolves to no factory. The `URLabDmEnvRpc.Build.cs` file still carries a Win64
branch listing `ws2_32` and friends; that is scaffolding for a Windows build that
does not currently happen, not evidence that one works.

<!-- REVISIT: transport scheme resolution is being made strict under the blocker
     work. Today a grpc:// endpoint on a platform with no factory fails without
     naming the reason; once that lands, this should say what the error looks
     like. -->

## Building gRPC

The plugin links a locally built static gRPC, pinned to v1.62.0. Nothing is
installed system-wide; everything lands under `third_party/install/grpc` as
`include/` and `lib/*.a`, which is where `URLabDmEnvRpc.Build.cs` looks for it.

`third_party/grpc/build.sh` does the clone and the CMake build, but running it
with the host compiler produces libraries the plugin cannot link. gRPC and
protobuf expose `std::string` across their public API, and Unreal links against
libc++, not libstdc++, so the two ABIs have to match. The build must therefore run
under Unreal's own toolchain:

```bash
TC=$UE/Engine/Extras/ThirdPartyNotUE/SDKs/HostLinux/Linux_x64/<clang-dir>/x86_64-unknown-linux-gnu

export CC="$TC/bin/clang"
export CXX="$TC/bin/clang++"
export CFLAGS="--sysroot=$TC"
export CXXFLAGS="--sysroot=$TC -stdlib=libc++ \
                 -Wno-missing-template-arg-list-after-template-kw -Wno-error"
export LIBRARY_PATH="$TC/lib64"

cd third_party/grpc && ./build.sh
```

Each of those is load-bearing:

- `--sysroot` points at Unreal's bundled sysroot rather than the host's. A modern
  host `<stdlib.h>` redirects `strtol` and `sscanf` to `__isoc23_*` symbols that
  Unreal's link-time libc does not have, and the plugin link then fails with an
  undefined `__isoc23_strtol`.
- `-stdlib=libc++` gives the C++ translation units the same ABI the plugin uses.
- `LIBRARY_PATH` is what lets the script's hard-coded `-lc++abi` resolve.
- The warning suppression covers gRPC 1.62's use of `.template` without an
  argument list, which recent clang rejects by default.

`build.sh` also exports `CMAKE_POLICY_VERSION_MINIMUM=3.5` for itself, since CMake
4 rejects the pre-3.5 minimums in gRPC 1.62's bundled dependencies.

The build takes somewhere between fifteen and forty minutes. Note that
`third_party/build_all.sh` does **not** build gRPC and
`URLab.Build.cs`'s `ThirdPartyPackages` list does not include it, so the staleness
check that covers MuJoCo, CoACD and libzmq does not cover this one. If the install
directory is absent, `Build.cs` adds no gRPC libraries and the module fails to
link, so build gRPC before enabling the module rather than after.

## Generated protobuf

The generated C++ is checked in under `Source/URLabDmEnvRpc/Private/GenProto/`, so
no `protoc` run is needed for an ordinary build. The `.proto` sources are tracked
beside it under `Private/Proto/`, and the bridge vendors its own generated Python
stubs of the same protos. `wire/regen_proto.sh` regenerates both sides, and
`wire/check_descriptors.py` asserts the two repos' descriptors are byte-identical;
`wire/toolchain.lock` records the versions they were produced with.

## What is not implemented

The native `dm_env_rpc` `step` handlers are stubs. The intended mapping — pose
transforms as action tensors, camera images as observation tensors, so a stock
`dm_env_rpc` client could drive the server without knowing about URLab — is not
built. Everything today goes through the extension tunnel described above.
