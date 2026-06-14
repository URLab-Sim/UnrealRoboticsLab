# Unreal Robotics Lab

Welcome to the documentation for **Unreal Robotics Lab (URLab)** — an Unreal Engine plugin that embeds the [MuJoCo](https://github.com/google-deepmind/mujoco) physics engine directly into the editor and runtime.

The plugin gives you research-grade contact dynamics alongside Unreal's rendering, Blueprint scripting, and editor tooling. Import a robot from MJCF XML (or build one in the component hierarchy), press Play, and MuJoCo handles the physics on a dedicated thread while Unreal handles everything visual.

> **Paper:** [Unreal Robotics Lab: A High-Fidelity Robotics Simulator with Advanced Physics and Rendering](https://arxiv.org/abs/2504.14135) — ICRA 2026

??? note "BibTeX Citation"
    ```bibtex
    @inproceedings{embleyriches2026urlab,
      title     = {Unreal Robotics Lab: A High-Fidelity Robotics Simulator with Advanced Physics and Rendering},
      author    = {Embley-Riches, Jonathan and Liu, Jianwei and Julier, Simon and Kanoulas, Dimitrios},
      booktitle = {IEEE International Conference on Robotics and Automation (ICRA)},
      year      = {2026},
      url       = {https://arxiv.org/abs/2504.14135}
    }
    ```

---

## Python (driving URLab from Python)

| Doc | What it covers |
|-----|----------------|
| [Getting Started](python/getting_started.md) | Connect, author a scene, run PIE, send control, tune gains, work with cameras, add a policy |
| [API Reference](python/api.md) | Full reference for `URLabClient` and the `scene` / `sim` / `runtime` / `outliner` / `debug` / `viewport` / `recording` / `replay` namespaces |
| [Bridge install + dashboard](python/bridge.md) | Python middleware: install, DearPyGui dashboard, pretrained policies, ROS 2 bridge |
| [Gym Environment](python/gym_environment.md) | `URLabEnv(gym.Env)` wrapper for users integrating with their own training pipelines |
| [Policy Registry](python/policy_registry.md) | Bundled pretrained policies; adding new ones to the registry |
| [Controllers and Gains](python/controllers_and_gains.md) | `ue_controller` vs `raw`, decimation, PD gain updates from Python |
| [Recording and Replay](python/recording_replay.md) | Capture episodes, save to disk, play back at wall-clock speed |
| [Entities](python/entities.md) | Non-articulation dynamic objects in step replies and PUB streams |

## URLab Plugin (Unreal-side)

| Doc | What it covers |
|-----|----------------|
| [Getting Started](getting_started.md) | UE plugin installation, first simulation, in-editor control methods |
| [Features](features.md) | Complete feature reference |
| [Roadmap](roadmap.md) | Planned features, known gaps, and future direction |
| [Architecture](architecture.md) | Subsystem design, threading model, compilation pipeline |
| [MJCF Import](guides/mujoco_import.md) | Importing MuJoCo XML models into Unreal |
| [Geometry & Collision](guides/geometry_authoring.md) | Primitives, mesh geoms, Quick Convert, heightfields |
| [Controller Framework](guides/controller_framework.md) | PD, keyframe, and custom controllers |
| [Debug Visualization](guides/debug_visualization.md) | Hotkey-driven overlays: contacts, joints, islands, segmentation, muscle/tendon tubes |
| [Interactive Perturbation](guides/perturbation.md) | Mouse-driven body drag: select, translate, rotate — simulate-compatible gestures |
| [Camera Capture Modes](guides/camera_capture_modes.md) | Per-camera RGB / depth / semantic + instance segmentation |
| [Possession & Twist Control](guides/possession_twist.md) | WASD control, spring arm camera |
| [Scripting with Blueprints](guides/blueprint_reference.md) | Hotkeys, API usage, scripting workflows |
| [Networking](guides/networking.md) | UE-side architecture: step modes (`live` / `direct` / `puppet`), transports (ZMQ / SHM), streaming wire format, ROS 2 bridge |
| [Step Server Protocol](reference/step_server.md) | Wire-protocol reference for the REP socket on port 5559 |

---

## Third-Party Software

| Library | License | Role |
|---------|---------|------|
| [MuJoCo](https://github.com/google-deepmind/mujoco) | Apache 2.0 | Physics simulation engine |
| [CoACD](https://github.com/SarahWeiii/CoACD) | MIT | Convex approximate decomposition |
| [libzmq](https://zeromq.org) | MPL 2.0 | High-performance messaging |

Full license texts are in `ThirdPartyNotices.txt`.

---

## Contributing

URLab welcomes contributions. See the repo's [CONTRIBUTING.md](https://github.com/URLab-Sim/UnrealRoboticsLab/blob/main/CONTRIBUTING.md) for the workflow, and use the [issue templates](https://github.com/URLab-Sim/UnrealRoboticsLab/issues/new/choose) for bug reports and feature requests.
