# Installation

<span class="os-tabs-linked" hidden></span>

Getting URLab running is three steps: install Unreal Engine, create a project,
then add the plugin and build it. Pick your OS once at the top; the rest of the
page follows.

**Before you start, you will need:**

- **Unreal Engine 5.7.** This is the only tested and supported version. The
  plugin code builds on earlier UE5 versions, but the bundled assets (dashboard
  UI, materials, input mappings) were saved in 5.7 and do not load on older
  versions. **Avoid 5.8 for now** — it has regressions with the Vulkan drivers on
  Linux that break rendering; stay on 5.7 until that is resolved.
- **CMake 3.24+** to build URLab's native dependencies (MuJoCo, CoACD, libzmq).
- **Python 3.11+** (optional), only if you drive URLab from Python.

!!! note "Prebuilt binaries are on the way"
    Today you build the dependencies from source. Precompiled Windows and Linux
    releases are planned (see the [Roadmap](roadmap.md)), which will remove the
    dependency build step.

## 1. Install Unreal Engine

=== "Windows"

    Install Unreal Engine 5.7 through the Epic Games Launcher. You also
    need **Visual Studio 2022 (17.10+) or 2025**, or **JetBrains Rider**, with the
    *Game development with C++* workload, to compile the plugin.

=== "Linux"

    Download the precompiled **Unreal Engine 5.7** Linux engine from your Epic
    account at <https://www.unrealengine.com/linux> and extract it. This guide
    refers to the extract root as `$UE_ROOT`.

    Alternatively, build from source via the
    [EpicGames/UnrealEngine](https://github.com/EpicGames/UnrealEngine) GitHub
    repository (checkout the `5.7` branch). Access requires
    [linking your Epic and GitHub accounts](https://www.unrealengine.com/en-US/ue-on-github).
    Do not use the `5.8` or `release` branches — see the version note above.

    !!! warning "The precompiled Linux engine omits some build scripts"
        As of UE 5.7.4 the Linux binary does not ship
        `Engine/Build/BatchFiles/Linux/GenerateProjectFiles.sh`. You do not need
        it: build the editor target directly with `Build.sh` (URLab's
        `Scripts/build_and_test_linux.sh` wraps this), or open the `.uproject` and
        let the editor rebuild. If you want IDE project files, JetBrains Rider can
        open a `.uproject` directly.

    On Ubuntu 22.04, install a recent CMake (the system one is 3.22, below CoACD's
    minimum):

    ```bash
    pip install --user "cmake>=3.24,<4"
    export PATH="$HOME/.local/bin:$PATH"
    ```

=== "macOS"

    macOS is not supported yet. It is on the [Roadmap](roadmap.md). If you would
    use it, open an issue so we can gauge demand.

## 2. Create a project

URLab is a C++ plugin, so it needs a host project with C++ enabled.

- Create a new project from a **C++ template**, or
- If you have a Blueprint project, convert it by adding one C++ class
  (**Tools > New C++ Class**, accept the default).

A Blueprint-only project will not compile the plugin. You do not need to write
any C++ yourself; the project just needs to be set up for it.

## 3. Add the URLab plugin

**Clone into the project's `Plugins/` folder, with submodules.** The
dependencies are git submodules pinned to exact commits, so clone recursively:

```bash
cd "YourProject/Plugins"
git clone --recurse-submodules https://github.com/URLab-Sim/UnrealRoboticsLab.git
```

**Build the native dependencies once:**

=== "Windows"

    ```powershell
    cd UnrealRoboticsLab/third_party
    .\build_all.ps1
    ```

=== "Linux"

    Point the build at your engine so the dependencies use Unreal's clang and
    libc++ (required for ABI compatibility):

    ```bash
    cd UnrealRoboticsLab/third_party
    ./build_all.sh --engine "$UE_ROOT"
    ```

This compiles MuJoCo, CoACD, and libzmq into `third_party/install/`. You only
repeat it when URLab bumps a dependency.

**Build the project:**

=== "Windows"

    Right-click your `.uproject`, choose **Generate Visual Studio project files**,
    then open the solution in Visual Studio or Rider and build. You can also build
    headlessly with `Scripts/build_and_test.ps1`.

=== "Linux"

    Build the editor target with URLab's helper (it calls Unreal's `Build.sh`,
    which the binary engine does ship):

    ```bash
    ./Scripts/build_and_test_linux.sh --engine "$UE_ROOT" --project /path/to/YourProject.uproject
    ```

    See [Building from Source](contributing/building.md) for the toolchain
    details, headless display setup, and runtime library staging.

!!! tip "Editor says modules are out of date or asks to rebuild?"
    If Unreal prompts that URLab is out of date or built with a different engine
    version, compile the project first, then reopen. Build with
    `Scripts/build_and_test.ps1` / `Scripts/build_and_test_linux.sh`, or from your
    IDE (we recommend **JetBrains Rider**). See
    [Troubleshooting](troubleshooting.md) if the build itself fails.

**Show plugin content.** In the Content Browser, open **Settings (gear icon)** and
enable **Show Plugin Content** so the dashboard UI and assets appear.

## Next steps

- **[Quickstart](quickstart.md)** imports a robot and runs your first simulation.
- **[Python & External Control](python/index.md)** sets up the Python client.
- **[Troubleshooting](troubleshooting.md)** covers the common setup snags.
