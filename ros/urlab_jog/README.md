# urlab_jog

Jog a URLab articulation from ROS 2 using only standard tooling. The slider
surface is the stock `joint_state_publisher_gui`; there is no custom node and no
relay. This directory ships only launch and rviz configuration plus these docs.

## How it works

```
joint_state_publisher_gui  ->  /<art>/joint_command  ->  URLab (applies control)
URLab  ->  /tf , /<art>/joint_states  ->  rviz (RobotModel + TF, real motion)
```

- `joint_state_publisher_gui` reads the URDF (`/robot_description`) and builds one
  slider per non-fixed joint, with limits taken from the URDF. It publishes
  `sensor_msgs/JointState`. The launch file remaps that output off the default
  `/joint_states` onto `/<art>/joint_command`, which URLab subscribes to and
  applies as control in Live mode. No relay node is needed: URLab accepts the
  standard `JointState` command directly.
- `robot_state_publisher` serves the URDF on `/robot_description` for rviz. URLab
  publishes the authoritative `/tf` (parent `world`, child `<art>/<body>`) from
  the live sim, so this node's own `/tf` output is remapped to a dead topic to
  avoid a two-parent TF tree.
- `rviz2` shows the robot with the standard RobotModel display (from
  `/robot_description` + `/tf`), so you see the actual sim motion, not the slider
  echo. The bundled `rviz/franka.rviz` presets RobotModel + TF with fixed frame
  `world`.

## Prerequisites

1. `joint_state_publisher_gui` is not in the base ROS env. Add it once
   (`robot_state_publisher` and `rviz2` are already present from the desktop
   install):

   ```powershell
   pixi add --manifest-path C:\dev\urlab_ros2_env\pixi.toml ros-lyrical-joint-state-publisher-gui
   ```

2. A URDF exported for the articulation. `joint_state_publisher_gui` and rviz's
   RobotModel both require it, and its names must line up with what URLab
   publishes:

   - URDF **joint names** must match URLab's joint names (the `name[]` URLab
     publishes on `/<art>/joint_states`, e.g. `joint1` .. `joint7`). The command
     `JointState` is matched to the sim by joint name, so a mismatch silently
     drives the wrong joints or nothing.
   - URDF **link names** must match URLab's TF frames `<art>/<body>` (e.g.
     `franka/panda_link0`), because rviz's RobotModel places each link by looking
     up its TF frame by name. The root/fixed frame is `world`.

   URLab is the naming authority (`FMjCanonicalName`); export the URDF to match.
   `ros2 topic echo --once /<art>/joint_states` and `ros2 topic echo --once /tf`
   against a running URLab show the exact joint and frame names to target.

3. Grant ROS control ownership of the art. URLab gates control writes per
   articulation; a ROS command is only applied when the art is owned by source
   `ros:urlab`, otherwise it is silently ignored. Claim it once from the bridge
   environment (`Plugins/URLab_Bridge`):

   ```bash
   uv run python -c "from urlab_client import URLabClient; c=URLabClient(); c.connect(); print(c.runtime.claim_control(articulation='franka', source='ros:urlab'))"
   ```

   Release later with
   `c.runtime.release_control(articulation='franka', source='ros:urlab')`.

## Launch (against a running URLab + Franka in Live mode)

`ros2 launch` accepts a direct file path, so no colcon build is needed:

```powershell
# Windows (from this directory)
pixi run --manifest-path C:\dev\urlab_ros2_env\pixi.toml `
  ros2 launch launch\franka_jog.launch.py art:=franka urdf:=C:\path\to\franka.urdf
```

```bash
# Linux (with the ROS env active, from this directory)
ros2 launch launch/franka_jog.launch.py art:=franka urdf:=/path/to/franka.urdf
```

Move a slider and the Franka tracks it in both the sim and rviz.

### Arguments

- `art`      articulation name / topic namespace (default `franka`); commands go
  to `/<art>/joint_command`.
- `urdf`     path to the articulation's URDF (required).
- `rviz`     rviz2 config path (default: the bundled `rviz/franka.rviz`).
- `use_rviz` set `false` to launch the sliders without rviz.

## Verify without moving anything

```bash
ros2 topic echo --once /franka/joint_command   # sliders publish here
ros2 topic echo --once /franka/joint_states    # URLab real state
ros2 topic echo --once /tf                      # URLab TF tree (frame names)
```

## Notes

- Only stock nodes are used: `joint_state_publisher_gui`,
  `robot_state_publisher`, `rviz2`. No relay, no custom GUI.
- The `/<art>/joint_command` `JointState` command input is served by URLab's ROS
  transport. If commands do not move the robot, check in order: the art is
  claimed for `ros:urlab` (step 3), URLab is in Live mode, and the URDF joint
  names match `/<art>/joint_states`.
- For a mobile base, `cmd_vel` (`geometry_msgs/Twist`) is a separate surface
  covered by `teleop_twist_keyboard`; it is out of scope for this joint-jog tool.
