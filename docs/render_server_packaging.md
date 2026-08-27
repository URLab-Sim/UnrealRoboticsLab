# Packaging the fast-path render server

How to cook a standalone Unreal render server that renders MuJoCo cameras on demand,
driven remotely from Python (see the bridge's `docs/render_server.md` for the client).

## Cook + stage (Windows)

From the host project (the `.uproject` that contains this plugin):

```powershell
& "$Engine\Engine\Build\BatchFiles\RunUAT.bat" BuildCookRun `
  -project="$Project\url_proj.uproject" -noP4 -platform=Win64 `
  -clientconfig=Development -cook -build -stage -pak -iostore -prereqs -nodebuginfo -utf8output
```

Output: `Saved/StagedBuilds/Windows/url_proj.exe` (the launcher; the real binary is under
`.../url_proj/Binaries/Win64/`). Re-run after any C++ change to the plugin.

`Config/DefaultGame.ini` force-cooks the URLab materials
(`+DirectoriesToAlwaysCook=(Path="/UnrealRoboticsLab/Materials")`) — they are referenced
dynamically, not from a map, so without this they are missing in the cooked build.

## Launching

```bash
url_proj.exe <lit-map> -URLabDrive=push -URLabModel=<scene.mjb> -URLabCaps=serve,cameras \
             -URLabScene=cammax=0 -RenderOffScreen -nosplash -abslog=server.log
```

| flag | meaning |
|------|---------|
| `<lit-map>` | a **lit** level — SkyLight + reflection captures — e.g. `/Game/FirstPerson/Lvl_FirstPerson`. An unlit map (`/Engine/Maps/Entry`) leaves metallic PBR looking flat. |
| `-URLabModel=<file>` | the model to load (version-matched `.mjb`; format from extension). (formerly `-URLabFastMjb=<file>`) |
| `-URLabDrive=push` | forced/eval regime: the bridge serves `fastpath_render`; cameras are manual-capture-only. (formerly `-URLabFastForcedOnly`) |
| `-URLabCaps=serve,cameras` | `serve` brings up the bridge + listeners; `cameras` enables camera capture. (formerly `-URLabFastCameras`) |
| `-URLabScene=cammax=N` | per-camera height cap. `0` = honour the model's `<camera resolution>` exactly. Any `N>0` caps height to `N` (downscaling, keeping aspect). Default cap is 480 (`AMjRenderer::CameraMaxHeight`). (formerly `-URLabFastCamMaxHeight=N`) |
| `-RenderOffScreen` | headless rendering |

### Resolution

Set per-camera resolution in the model (`<camera resolution="1280 960">`) and launch with
`-URLabScene=cammax=0`. Changing resolution is then just an MJB recompile — no recook.

### Scalability vs fidelity

`-ExecCmds="sg.ShadowQuality 0; sg.PostProcessQuality 0"` forces LOW scalability for
comparable perf benchmarks, but it disables screen-space reflections and post-processing,
which flattens metallics. For a fidelity/viewer run, use full scalability
(`sg.PostProcessQuality 3; sg.ReflectionQuality 3; r.SSR.Quality 3`).

## Model (MJB) compilation

The server needs a `.mjb` matching Unreal's MuJoCo version. Compile with the matching
`mjbcompile` from the scene directory (so mesh/texture paths resolve):

```bash
mjbcompile scene.xml scene.mjb     # prints "ver=3011001" — must match the UE MuJoCo
```
