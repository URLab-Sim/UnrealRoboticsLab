# urlab_moveit — MoveIt planning for the URLab Franka

Motion-plan the arm in rviz and execute the trajectory on the running URLab sim.
Uses stock MoveIt; the only custom piece is a small FollowJointTrajectory bridge
that streams planned points onto `/<art>/joint_command` (so no ros2_control stack
is needed on the sim side).

## Layout

```
config/
  franka.srdf            auto-generated (MuJoCo-sampled disable-collisions + keyframe states)
  kinematics.yaml        KDL IK for the panda_arm group
  joint_limits.yaml      velocity (URDF) + acceleration limits for time-parameterization
  ompl_planning.yaml     OMPL planners (RRTConnect default)
  moveit_controllers.yaml maps panda_arm -> FollowJointTrajectory
scripts/
  generate_srdf.py       regenerate the SRDF from a MuJoCo model
  trajectory_bridge.py   FollowJointTrajectory -> /<art>/joint_command
launch/
  franka_moveit.launch.py move_group + rsp + static tf + bridge + rviz
rviz/
  moveit.rviz            MotionPlanning display
```

## Prerequisite (one-time, user runs)

MoveIt is not in the env yet. Add it to the pixi ROS env:

```
pixi add --manifest-path C:\dev\urlab_ros2_env\pixi.toml ros-lyrical-moveit
```

(That metapackage pulls `move_group`, the OMPL planner, KDL kinematics, and the
rviz MotionPlanning plugin.)

## Regenerate the SRDF (optional; already committed)

Run under the bridge env (`uv run`), pointing at the MuJoCo model whose link
names match the exported URDF:

```
uv run python scripts/generate_srdf.py \
  --xml C:\dev\menagerie\franka_emika_panda\panda_ros_demo.xml \
  --out config\franka.srdf --samples 20000
```

Disabled pairs come from real MuJoCo collision sampling (Adjacent / Default /
Always / Never), so they match the physics rather than an approximate mesh check.

## Bring-up

1. Start the sim in Live mode (imports the Franka, unpauses, streams). From the
   bridge repo:
   ```
   uv run python <scratch>/m1_sensors_setup.py
   ```
2. Launch MoveIt (path args must be Windows 8.3 short paths because the project
   path contains a space):
   ```
   pixi run --manifest-path C:\dev\urlab_ros2_env\pixi.toml ros2 launch \
     <short>\launch\franka_moveit.launch.py urdf:=<short>\franka\model.urdf
   ```
3. In rviz's **MotionPlanning** panel: drag the goal marker (or pick the `home`
   named state), **Plan**, then **Plan & Execute** — the arm follows the
   trajectory in the UE sim.

## Notes

- URDF link names are bare (`link0..link7`, `hand`); the launch anchors them
  under `world` with a static transform and feeds joint state from
  `/<art>/joint_states`.
- Everything runs on **sim time** (`/clock`), so trajectory timing matches the
  (possibly non-real-time) sim.
- The gripper (`finger_joint1/2`) is tendon-driven with no direct joint actuator,
  so it is not a MoveIt execution group yet — arm planning only for now.
