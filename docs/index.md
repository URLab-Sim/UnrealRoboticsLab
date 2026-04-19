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

## Guides

| Guide | What it covers |
|-------|---------------|
| [Getting Started](getting_started.md) | Installation, first simulation, control methods |
| [Features](features.md) | Complete feature reference |
| [MJCF Import](guides/mujoco_import.md) | Importing MuJoCo XML models into Unreal |
| [Geometry & Collision](guides/geometry_authoring.md) | Primitives, mesh geoms, Quick Convert, heightfields |
| [Controller Framework](guides/controller_framework.md) | PD, keyframe, and custom controllers |
| [Debug Visualization](guides/debug_visualization.md) | Hotkey-driven overlays: contacts, joints, islands, segmentation, muscle/tendon tubes |
| [Interactive Perturbation](guides/perturbation.md) | Mouse-driven body drag: select, translate, rotate — simulate-compatible gestures |
| [Camera Capture Modes](guides/camera_capture_modes.md) | Per-camera RGB / depth / semantic + instance segmentation |
| [Possession & Twist Control](guides/possession_twist.md) | WASD control, spring arm camera |
| [Scripting with Blueprints](guides/blueprint_reference.md) | Hotkeys, API usage, scripting workflows |
| [ZMQ Networking & ROS 2](guides/zmq_networking.md) | ZMQ transport, topics, camera streaming |
| [URLab Bridge](guides/policy_bridge.md) | Python middleware, RL policies, remote control |
| [Architecture](architecture.md) | Subsystem design, threading model, compilation pipeline |

---

## API Reference

The [API Reference](api/index.md) is auto-generated from C++ headers on each build. It covers every class, struct, and enum in the plugin.

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
