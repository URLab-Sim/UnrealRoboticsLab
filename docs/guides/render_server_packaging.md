# Packaging a render server

Cook and stage the host Unreal project that includes the plugin. The staged launcher can run a passive render workflow; its source arrives through a `RenderClient` owner attach and install operation. The current launch selector is `-URLabWorkflow=render` (`-URLabDrive` is rejected). See [workflow flags](../reference/render_server_flags.md).

For example, after staging a Linux `URLabTest` project:

```bash
Saved/StagedBuilds/Linux/URLabTest.sh /Game/FastPath/FastPathRender \
  -URLabWorkflow=render -URLabScene=cammax=0 \
  -RenderOffScreen -nosplash -unattended -stdout
```

Use a lit map when appearance matters. `cammax=0` permits the camera resolution authored in the model. The optional gRPC module is Linux-only; it carries URLab render extensions with network data, while ZMQ provides the standard command endpoint. The bridge's [typed render guide](https://github.com/URLab-Sim/urlab_bridge/blob/main/docs/render_server.md) covers connecting, source installation and captures; its [pool guide](https://github.com/URLab-Sim/urlab_bridge/blob/main/docs/render_pool.md) covers independent workers. See [completion status](../architecture/completion_status.md) for the tested packaged combinations.
