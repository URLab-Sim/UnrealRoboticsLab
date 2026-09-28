# Sensors & Cameras

Read simulation state through MuJoCo sensors, and capture the scene through cameras that output photoreal RGB, depth, or segmentation masks.

## Sensors

A sensor reads a quantity from the simulation each step. URLab imports every MuJoCo sensor type from MJCF as a component under `SensorsRoot`, and you can add more in the Blueprint editor (set the sensor's **Target** to choose what it measures). See [Articulations](articulations.md).

Rather than memorise individual classes, think in categories:

- **Joint and tendon state.** Position, velocity, and force/torque for joints, tendons, and their limits.
- **Actuator state.** Commanded position, velocity, and force per actuator.
- **Body and frame state.** Position, orientation, axes, linear/angular velocity, and linear/angular acceleration of a body, site, or geom frame; subtree center of mass, linear velocity, and angular momentum.
- **Inertial measurement.** Accelerometer, gyroscope, velocimeter, magnetometer (the building blocks of an IMU).
- **Contact and touch.** Touch sensors, contact sensors, force and torque at a site, and tactile arrays.
- **Geometry queries.** Range finders, geom-to-geom distance, normals, and whether a point lies inside a site.
- **Energy and clock.** Kinetic and potential energy, simulation time.
- **Custom.** User and plugin sensors for values your own code computes.

### Reading sensors

Each sensor returns either a single scalar or a short vector. Read them by name from the articulation:

```cpp
float Touch = Robot->GetSensorScalar("fingertip_touch"); // scalar sensors
TArray<float> Force = Robot->GetSensorReading("wrist_force"); // vector sensors
float Angle = Robot->GetJointAngle("elbow"); // joint-position shortcut
```

Use `GetSensorScalar` for scalar readings (touch, joint position, clock) and `GetSensorReading` for vector readings (force, accelerometer, gyro). Both are Blueprint-callable. Live readouts also appear in the [Simulate Dashboard](dashboard.md), and sensor values stream to Python over the bridge (see [Python](../python/index.md)).

## Cameras

A `UMjCamera` renders the scene from a fixed viewpoint into a render target you can preview in the dashboard or stream to Python. Imported cameras keep their MuJoCo intrinsics: where the MJCF used `focal`, `focalpixel`, or `sensorsize`, the importer derives the field of view so the Unreal camera matches MuJoCo. See [Importing Robots](importing.md).

### Capture modes

Each camera has a `CaptureMode` property (`EMjCameraMode`), set per-camera at design time in the Details panel. It decides what the render target contains:

| Mode | Output |
|---|---|
| `Real` (default) | Photoreal RGB. Matches the viewport, respects post-process. |
| `Depth` | Linear scene depth, in centimetres. |
| `SemanticSegmentation` | A color tint per actor class. |
| `InstanceSegmentation` | A unique color tint per body. |

One camera produces one stream. To capture RGB plus depth plus masks from the same viewpoint, place several cameras at the same transform with different modes. Segmentation modes do not swap materials on the real meshes, so an RGB camera and a segmentation camera can stream the same frame at once without contaminating each other.

For `Depth`, also set `DepthNearCm` (default 10) and `DepthFarCm` (default 10000); the dashboard uses these to normalise the grayscale preview.

!!! note
    `CaptureMode` is read when streaming starts. To change the mode of a camera that is already running, toggle streaming off and on:

    ```cpp
    Cam->SetStreamingEnabled(false);
    Cam->CaptureMode = EMjCameraMode::Depth;
    Cam->SetStreamingEnabled(true);
    ```

### Previewing and streaming

The dashboard's camera panel shows every camera on the selected articulation, each rendering its configured mode live, with no extra setup. See [Simulate Dashboard](dashboard.md).

For headless or full-framerate capture, stream frames to Python over the bridge. Each camera can broadcast over ZMQ or shared memory; the consumer interprets the bytes per the camera's mode (RGBA for `Real`, float32 centimetres for `Depth`, BGRA tint for the segmentation modes). See [Python Policies](../python/policies.md) and the [bridge protocol](../reference/protocol.md).

!!! warning
    Camera capture modes are independent of the key-`6` viewport segmentation overlay in [Debug Visualization](debug.md). The hotkey overlay swaps real materials and therefore appears in any capture; the per-camera modes are the right tool for clean, persistent RGB, depth, or mask streams. Note that the segmentation tints are lit-shaded rather than flat, so they are good for visual debugging but not pixel-exact ground truth.

## Lidar point clouds

The lidar computation is engine-agnostic at its core: a `ULidarComponent` owns the beam grid, noise, blind zone, clustering, and publishing, and works standalone in a plain UE scene by raycasting the Unreal collision world from its own transform. `UMjLidarSensor` is the MuJoCo adapter you use in URLab scenes: it swaps the ray source for the simulation's ground truth — `mj_multiRay` against the compiled MuJoCo model on the physics thread — and reads the sensor pose from the mount body or site, so scans see exactly what MuJoCo sees. Attach it under a body or site of your robot, add a `UMjLidarPointCloudViz` component to the same actor (it auto-binds to the owner's first lidar component, or set **Source Sensor**), and the scans become a visible point cloud for inspecting beam coverage, occlusion, noise, and clustering. The viz is event-driven - no Tick - and every property takes effect live in PIE.

!!! note
    The MuJoCo sensor casts against MuJoCo geoms in the groups enabled by its **Geom Group Mask** (default `9`: group 0 primitives and floors plus group 3 collision hulls; visual-only group 2 is excluded so a mesh with both visual and collision geoms is not double-hit). A plain UE `StaticMeshActor` without any `Mj*` component is invisible to the lidar regardless of the mask — Quick Convert it first if it should be sensed. A bare `ULidarComponent` (no adapter) instead LineTraces the Unreal world on its **Trace Channel**, for lidar prototyping in scenes with no MuJoCo at all.

The **Backend** property chooses the renderer:

| Backend | Best for | Notes |
|---|---|---|
| `BatchedDebug` (default) | Quick checks | `DrawDebugPoint` calls, strided to **Batched Max Points** (20,000) so full scans cannot stall the frame rate. |
| `InstancedStaticMesh` | Full-resolution shape/occlusion checks | One cube instance per point (up to **Max Points**). |
| `Niagara` | Full-resolution checks with styling | Needs a point system asset with the four `MjLidar.*` user parameters; falls back to BatchedDebug when unset. |

The **Color Mode** property colorizes each point: `SingleColor` (fixed), `ByRange` (red near, blue far - the clearest view of occlusion boundaries), `ByHeight`, `ByElevationRing` (one color per scan ring, revealing the fan pattern), or `BySurfaceId` (discrete palette per hit surface, gray for misses).

**History Scans** overlays the last N scans (1 = latest only), which makes drift and noise visible; **Freeze** stops consuming new scans for close inspection. The small axis triplet marks the sensor origin, and aggregated targets get colored spheres at their centroids.

`Export Last Scan to PLY` writes the current cloud (history included) as ASCII PLY to `Saved/URLab/LidarScans/`, with positions in metres, ready for CloudCompare or meshlab.

!!! note
    The instanced backend colors points through Per-Instance Custom Data, so it needs a material that reads the slots: in the material editor, add three *Per Instance Custom Data* nodes (Custom Data Index 0, 1, 2), append them into an RGB vector wired to Base Color and Emissive, and assign the material to **Point Material**. Without it the instances render in the mesh's default gray - shapes and occlusion still check out, colors do not.
