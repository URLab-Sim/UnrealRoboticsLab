# UnrealRoboticsLab

UnrealRoboticsLab (URLab) embeds the [MuJoCo](https://github.com/google-deepmind/mujoco)
physics engine directly inside Unreal Engine 5. You author and simulate robots
with MuJoCo's accurate contact dynamics, and render them with Unreal's lighting,
materials, and cameras.

![URLab simulating a robot in Unreal Engine](images/hero.png)

## What you can do

- **Bring MuJoCo models into Unreal.** Drag an MJCF `.xml` file into the Content
  Browser and URLab builds a ready-to-simulate Blueprint, or turn any static
  mesh into a physics body with one component.
- **Simulate with MuJoCo, render with Unreal.** Physics runs on MuJoCo on its
  own thread; Unreal handles rendering, so you get accurate contacts and
  photorealistic output at the same time.
- **Control robots however you like.** Drive actuators from Blueprints, from the
  in-editor dashboard, or from Python and ROS 2 over the network.
- **Capture data.** Render RGB, depth, and segmentation from robot cameras, and
  record and replay full simulation episodes.

## Who it is for

URLab pairs MuJoCo physics with what Unreal does well: photorealistic rendering,
scene generation, NPCs, and the rest of the editor toolset. It is built for
evaluation, data generation, experimentation, and simulation.

You can train in URLab, but it runs one simulation at a time rather than
parallelising across thousands of environments on the GPU, so it is not the
fastest route for large-scale reinforcement learning. For that, train with a
dedicated massively parallel MuJoCo framework such as
[mjlab](https://github.com/mujocolab/mjlab) or
[mujoco_warp](https://github.com/google-deepmind/mujoco_warp), then evaluate the
resulting policies directly in URLab. Because both use MuJoCo, the
[Python bridge](python/index.md) lets you move between training there and
evaluating, recording, and visualising here.

## Get started

<div class="grid cards" markdown>

- **[Install URLab](installation.md)** sets up the plugin and its dependencies on
  Windows or Linux.
- **[Quickstart](quickstart.md)** imports a robot and runs your first simulation
  in a few minutes.
- **[Python & External Control](python/index.md)** drives the simulation from
  Python, for scripted control and running policies.
- **[Architecture](concepts/architecture.md)** explains how the pieces fit
  together under the hood.

</div>

## Project

URLab is open source under the Apache 2.0 license. See the
[Roadmap](roadmap.md) for where it is headed. If you use URLab in research,
please cite the [ICRA 2026 paper](https://arxiv.org/abs/2504.14135).
