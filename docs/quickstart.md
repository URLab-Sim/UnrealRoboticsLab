# Quickstart

This walks you from an installed plugin to a robot simulating in your level in a
few minutes. If you have not installed URLab yet, start with
[Installation](installation.md).

## 1. Import a robot

Get a MuJoCo model (the [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie)
is a good source) and drag its `.xml` file into the Content Browser.

On the first import, URLab offers to install the Python packages it uses for mesh
processing into Unreal's bundled Python, so no external setup is needed. URLab
then generates a Blueprint containing the robot's bodies, joints, actuators, and
sensors.

![Dragging an MJCF file into the Content Browser to generate a robot Blueprint](images/placeholder.svg)

See [Importing Robots](guides/importing.md) for the full import workflow, and
[Articulations](guides/articulations.md) for editing the generated Blueprint.

## 2. Set up the level

1. Place an **MjManager** actor in your level. It coordinates the simulation, one
   per level.
2. Drag your imported robot Blueprint into the level.

![A robot Blueprint and an MjManager placed in the level](images/placeholder.svg)

## 3. Press Play

Hit Play. MuJoCo starts simulating on its own thread while Unreal renders. The
**Simulate** dashboard appears, giving you actuator sliders, sensor readouts,
live camera feeds, and keyframe controls.

![The Simulate dashboard during play](images/placeholder.svg)

!!! note "No dashboard?"
    Make sure an MjManager is in the level with `bAutoCreateSimulateWidget`
    enabled, and that **Show Plugin Content** is on in the Content Browser
    settings.

## 4. Move the robot

Try any of these:

- **Dashboard.** Set the control source to UI, then drag the actuator sliders.
  See the [Simulate Dashboard](guides/dashboard.md) guide.
- **Blueprint.** Call the articulation's control functions (set actuator targets,
  read joints and sensors) from your own graphs. See
  [Controllers](guides/controllers.md).
- **Python.** Connect a client to step the simulation, send control, and run
  policies. See the [Python Quickstart](python/quickstart.md).

## Where to next

- [Importing Robots](guides/importing.md) and [Articulations](guides/articulations.md)
- [Sensors & Cameras](guides/sensors_cameras.md) for cameras and sensor readouts
- [Debug Visualization](guides/debug.md) for contact, joint, and collision overlays
- [Python & External Control](python/index.md) to drive everything from code
