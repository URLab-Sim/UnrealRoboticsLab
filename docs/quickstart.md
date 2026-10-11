# Quickstart

This walks you from an installed plugin to an authored robot in your level.
If you have not installed URLab yet, start with
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

1. Place an **MjManager** actor in your level as the authoring source.
2. Drag your imported robot Blueprint into the level.

![A robot Blueprint and an MjManager placed in the level](images/placeholder.svg)

## 3. Press Play

Hit Play to activate the world-owned MuJoCo simulation service for the authored
scene. Runtime simulation state and control belong to that service, not to
the manager or articulation actors.

## 4. Control the simulation

Use the companion bridge's `SimulationClient` against an already available
simulation. Connect, inspect the model generation, explicitly claim control,
then choose the clock and send controls or steps. See the
[Python overview](python/index.md) and [architecture](architecture/intended_design.md).

## Where to next

- [Importing Robots](guides/importing.md) and [Articulations](guides/articulations.md)
- [Sensors & Cameras](guides/sensors_cameras.md) for cameras and sensor readouts
- [Debug Visualization](guides/debug.md) for contact, joint, and collision overlays
- [Python & External Control](python/index.md) for the current service clients
