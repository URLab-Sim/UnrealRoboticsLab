# Simulation, render server and viewer

A process selects one workflow at launch with `-URLabWorkflow=simulation|render|viewer`. The old `-URLabDrive` and `-URLabSourceFind` flags are rejected. `-URLabClock=stepped|freerun` applies only to simulation.

| Workflow | Owns | Python client |
|---|---|---|
| `simulation` | MuJoCo model/data, clock, observations and control authority | `SimulationClient` after a control claim |
| `render` | Installed neutral scene, pose frames and exact camera capture jobs | `RenderClient` after an owner attach |
| `viewer` | Visible scene and frames from one owner or bound pose producer | `ViewerClient` after an owner attach |

Simulation control, render/viewer scene ownership and process reservation are independent. A passive connection grants none of them. A render or viewer source can come from XML plus assets, MJB, MJZ or a neutral scene. A `qpos` render frame uses MuJoCo kinematics to derive Cartesian transforms; it does not step physics. Neutral scenes and frames also permit a producer that does not use MuJoCo.

A viewer follows one active owner. Its owner can install a source, bind an advertised Cartesian pose stream and submit authorized interaction requests. Camera output from a simulation or viewer is configured by its owner, then consumed by an independent `OutputReader` using the exact advertised stream descriptor. A render server instead returns retained exact captures to a submitted job. See the [typed render guide](https://github.com/URLab-Sim/urlab_bridge/blob/main/docs/render_server.md) and [transport contract](../architecture/transport_design.md).

The renderer consumes neutral scene and frame values; the MuJoCo adapter performs model extraction and `qpos` to Cartesian conversion. See [intended design](../architecture/intended_design.md) and [completion status](../architecture/completion_status.md) for the verified boundaries.
