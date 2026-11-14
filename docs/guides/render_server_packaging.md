# Packaging a render server

Cook and stage the host Unreal project that includes the plugin. The staged launcher can run a passive render workflow; its source arrives through a `RenderClient` owner attach and install operation. The current launch selector is `-URLabWorkflow=render` (`-URLabDrive` is rejected). See [workflow flags](../reference/render_server_flags.md).

For example, after staging a Linux `URLabTest` project:

```bash
Saved/StagedBuilds/Linux/URLabTest.sh /Game/FastPath/FastPathRender \
  -URLabWorkflow=render -URLabNet=step=5559,grpc=50051 -URLabScene=cammax=0 \
  -RenderOffScreen -nosplash -unattended -stdout
```

Use a lit map when appearance matters. `cammax=<px>` caps the height of every installed camera (the width keeps its aspect); `cammax=0` keeps the resolution authored in the model. The render workflow applies no other `-URLabScene` key and refuses to boot if one is given; the viewer workflow applies none.

## Network exposure

The server binds `127.0.0.1` unless told otherwise, because the bridge has no authentication and grants model uploads. To serve other machines, bind every interface or one NIC explicitly, for example `-URLabNet=step=5559,grpc=50051,bind=0.0.0.0`, and restrict access at the network level. The registry and the connect metadata advertise the bound address (this machine's name for a wildcard bind). `URLAB_BIND_ADDRESS` and `BindAddress` in `LocalUnrealRoboticsLab.ini` set the same value.

## Boot failures and process lifetime

A headless boot that cannot serve exits with status 3 instead of running without a listener: a refused flag combination, a workflow activation failure, or a ZMQ or gRPC port that cannot be bound. The gRPC server does not share ports (`SO_REUSEPORT` is disabled), so a second server on a taken port fails rather than splitting connections. `-URLabNet=parent=<pid>` makes the server exit when that process is gone; the Python launcher passes its own pid, so a killed client cannot leave a server behind. `-URLabNet=id=<id>` sets the instance id reported in the connect metadata, which the Python launcher uses to confirm that the port it dials belongs to the process it started. The optional gRPC module is Linux-only; it carries URLab render extensions with network data, while ZMQ provides the standard command endpoint. The bridge's typed render guide (`docs/render_server.md` in [URLab_Bridge](https://github.com/URLab-Sim/URLab_Bridge)) covers connecting, source installation and captures; its pool guide (`docs/render_pool.md` in [URLab_Bridge](https://github.com/URLab-Sim/URLab_Bridge)) covers independent workers. See [completion status](../architecture/completion_status.md) for the tested packaged combinations.

## The render map

The plugin ships no map; the host project provides one. Any map works: the render service is started by `UMjRendererLauncher`, a world subsystem, so it boots in whichever map the process opens. The map contributes only the environment around the model; an empty level renders a black background around the model lights, while a lit level provides ambient environment lighting.

The default path is `/Game/FastPath/FastPathRender`. The Python launcher opens it unless told otherwise, and the editor launcher (`-URLabWorkflow=...` on the editor command line) switches to it, creating an empty level there if the project has none. To use another map, set `URLAB_MAP=/Game/Path/MyMap`, which both the packaging script and the Python launcher read, or pass `map_name="/Game/Path/MyMap"` to the Python launcher (or put the path first on the packaged command line, as above).

The map is reached by path, not by reference, so the cooker includes it only when listed in the host project's `Config/DefaultGame.ini`:

```ini
[/Script/UnrealEd.ProjectPackagingSettings]
+MapsToCook=(FilePath="/Game/FastPath/FastPathRender")
+DirectoriesToAlwaysCook=(Path="/UnrealRoboticsLab/Materials")
```

The plugin's own `Config/DefaultGame.ini` already asks for its materials; repeating the rule in the host project keeps them cooked when the host overrides its packaging settings. If they are missing at runtime the log says `required plugin assets did not load`, and segmentation is not advertised.

## Packaging script (Linux)

`Scripts/package_render_server_linux.sh` does all of the above for a host project and then runs `RunUAT BuildCookRun`:

```bash
Plugins/UnrealRoboticsLab/Scripts/package_render_server_linux.sh \
  /abs/path/Host.uproject [out dir] [Development|Test|Shipping]
```

It builds the host editor target, creates an empty render map (`new_level` with `DefaultGameMode=AGameModeBase`) if it does not exist (headless, through the `pythonscript` commandlet), adds the two cook rules if they are missing, and cooks, stages and paks with IoStore. Without an out dir the build stays in `Saved/StagedBuilds/Linux`; with one it is archived to `<out dir>/Linux`. `UE_ROOT` selects the engine (default `~/Documents/UE/UnrealEngine`). `URLAB_DDC=Cold` cooks against a new, empty derived-data cache (the engine's `-ddc=Cold` graph) instead of reusing the machine's cache, so every shader and derived asset is rebuilt as on a fresh machine; expect a much longer cook. Check the result with `Scripts/smoke_packaged_linux.sh /abs/path/to/Host.sh`. The smoke test runs from a synced URLab_Bridge checkout (`uv sync --extra grpc --extra dev`); it looks for one next to the plugin checkout, or set `URLAB_BRIDGE=/path/to/URLab_Bridge`.

The packaged process needs no editor-only libraries: CoACD (convex decomposition) and the editor scripting plugin are left out of runtime targets. MuJoCo's MJZ archive decoder is part of `libmujoco`, which is always staged; the log line `MuJoCo MJZ decoder available` confirms it at startup.
