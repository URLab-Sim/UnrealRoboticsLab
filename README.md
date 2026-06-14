[![Documentation](https://img.shields.io/badge/Documentation-blue.svg)](https://urlab-sim.github.io/UnrealRoboticsLab/)
[![arXiv](https://img.shields.io/badge/arXiv-2504.14135-b31b1b.svg)](https://arxiv.org/abs/2504.14135)
[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](LICENSE)
[![MuJoCo](https://img.shields.io/badge/MuJoCo-3.9%2B-green.svg)](https://github.com/google-deepmind/mujoco)
[![Unreal Engine](https://img.shields.io/badge/Unreal_Engine-5.7%2B-black.svg)](https://www.unrealengine.com)

# URLab: MuJoCo Physics in Unreal Engine

![URLab](docs/images/hero.png)

UnrealRoboticsLab (URLab) is an Unreal Engine 5 plugin that embeds the
[MuJoCo](https://github.com/google-deepmind/mujoco) physics engine directly into
the editor and runtime. You simulate robots with MuJoCo's accurate contact
dynamics and render them with Unreal's lighting, materials, and cameras.

> **Full documentation lives at
> [urlab-sim.github.io/UnrealRoboticsLab](https://urlab-sim.github.io/UnrealRoboticsLab/).**
> Start there for installation, guides, the Python API, and everything else. This
> README is only a summary.

## What you can do

- **Import MuJoCo models.** Drag an MJCF `.xml` into the Content Browser and URLab
  builds a ready-to-simulate Articulation Blueprint, with support for joints,
  actuators, tendons, muscles, and flexcomp. Or turn any static mesh into a physics
  body with a single component.
- **Simulate with MuJoCo, render with Unreal.** Physics runs on its own thread while
  Unreal renders, so you get accurate contacts and photorealistic output together.
- **Control robots however you like.** Drive actuators from Blueprints, the in-editor
  dashboard, or Python and ROS 2 over the network, with PD controllers and live gain
  tuning.
- **Capture sensor data.** Render RGB, depth, and segmentation from robot cameras with
  real camera intrinsics, alongside MuJoCo's full sensor suite.
- **Record and replay.** Capture entire simulation episodes and play them back
  deterministically.
- **Debug visually.** Body-island and segmentation overlays, tendon and muscle
  rendering, collision overlays, and mouse-driven body perturbation.

## Quick install

URLab is a C++ plugin, so your project must be a C++ project. Clone it into your
project's `Plugins/` folder with submodules, build the native dependencies once,
then build your project:

```bash
cd "YourProject/Plugins"
git clone --recurse-submodules https://github.com/URLab-Sim/UnrealRoboticsLab.git
cd UnrealRoboticsLab/third_party
./build_all.sh        # or .\build_all.ps1 on Windows
```

See the [Installation guide](https://urlab-sim.github.io/UnrealRoboticsLab/installation/)
for the full Windows and Linux walkthrough, then the
[Quickstart](https://urlab-sim.github.io/UnrealRoboticsLab/quickstart/).

## Documentation

Full documentation is at
[urlab-sim.github.io/UnrealRoboticsLab](https://urlab-sim.github.io/UnrealRoboticsLab/).

| Page | Covers |
|------|--------|
| [Installation](https://urlab-sim.github.io/UnrealRoboticsLab/installation/) | Windows and Linux setup |
| [Quickstart](https://urlab-sim.github.io/UnrealRoboticsLab/quickstart/) | Import a robot and run a simulation |
| [Guides](https://urlab-sim.github.io/UnrealRoboticsLab/guides/importing/) | Importing, articulations, sensors, cameras, controllers, debug |
| [Python & External Control](https://urlab-sim.github.io/UnrealRoboticsLab/python/) | Drive URLab from Python, run and evaluate policies |
| [Architecture](https://urlab-sim.github.io/UnrealRoboticsLab/concepts/architecture/) | How the pieces fit together |
| [Roadmap](https://urlab-sim.github.io/UnrealRoboticsLab/roadmap/) | Where URLab is headed |

## Python integration

URLab talks to external systems over ZMQ. The companion
[urlab_bridge](https://github.com/URLab-Sim/urlab_bridge) package (separate
repository) provides Python middleware for remote control, policy deployment, and
ROS 2 bridging.

## Requirements

- Unreal Engine 5.7+, Windows (Win64) with Visual Studio 2022/2025, or Linux
  (x86_64) with UE's bundled clang.
- CMake 3.24+ to build the dependencies.
- MuJoCo, CoACD, and libzmq are bundled as submodules and built from source.
- Python 3.11+ (optional), for `urlab_bridge`.

## Contributing

Contributions are welcome. See [CONTRIBUTING.md](CONTRIBUTING.md), and the
[Contributing docs](https://urlab-sim.github.io/UnrealRoboticsLab/contributing/building/)
for building from source, the codegen, and bumping MuJoCo. Since Unreal projects
cannot use standard CI, each PR should include proof of a local build and passing
tests.

## Citation

If you use URLab in your research, please cite the
[ICRA 2026 paper](https://arxiv.org/abs/2504.14135):

```bibtex
@inproceedings{embleyriches2026urlab,
  title     = {Unreal Robotics Lab: A High-Fidelity Robotics Simulator with Advanced Physics and Rendering},
  author    = {Embley-Riches, Jonathan and Liu, Jianwei and Julier, Simon and Kanoulas, Dimitrios},
  booktitle = {IEEE International Conference on Robotics and Automation (ICRA)},
  year      = {2026},
  url       = {https://arxiv.org/abs/2504.14135}
}
```

## License

Copyright 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version
2.0. See [LICENSE](LICENSE).

**Disclaimer:** UnrealRoboticsLab is an independent software plugin. It is NOT
affiliated with, endorsed by, or sponsored by Epic Games, Inc. "Unreal" and
"Unreal Engine" are trademarks or registered trademarks of Epic Games, Inc. in
the US and elsewhere. This plugin incorporates third-party software: MuJoCo
(Apache 2.0), CoACD (MIT), and libzmq (MPL 2.0). See
[ThirdPartyNotices.txt](ThirdPartyNotices.txt) for details.
