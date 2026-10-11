# Running policies

The previous policy examples used the removed `URLabClient` articulation and Gym/task wrapper APIs. They are no longer runnable. Gym/task wrappers and recording/replay are outside the completed architecture scope.

For a custom control loop, use the current [Python overview](index.md) and `SimulationClient` against an already available simulation. For Python-owned MuJoCo physics with Unreal as viewer, see the companion bridge's runnable `examples/owner_viewer_bunny.py`. The [completion status](../architecture/completion_status.md) records what has been verified.
