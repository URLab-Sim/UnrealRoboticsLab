"""
End-to-end integration test for the three ROS map providers.

Boots a test scene via the URLab RPC, starts PIE, and verifies that
/urlab/obstacle_cloud, /map, and /octomap_binary are publishing.

Usage:
  uv run ros/urlab_ros_ws/scripts/test_maps.py [--ue-address tcp://localhost]

Requires:
  - UE editor running with the URLab plugin loaded (no PIE yet — the
    script calls begin_pie)
  - ROS 2 sourced (ros2 CLI on PATH)
  - uv environment with urlab_client installed
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
import time
from pathlib import Path
from typing import Optional

# --- helpers -----------------------------------------------------------

def ros2(*args: str, timeout_s: float = 10.0) -> subprocess.CompletedProcess:
    """Run ``ros2 <args>`` and return the completed process."""
    return subprocess.run(
        ["ros2", *args],
        capture_output=True, text=True, timeout=timeout_s,
    )


def require_ros2() -> None:
    """Fail fast when ros2 isn't on PATH."""
    r = subprocess.run(["ros2", "topic", "list"], capture_output=True, text=True, timeout=5.0)
    if r.returncode != 0:
        sys.exit(f"ros2 not available; source ROS 2 first.\nstderr: {r.stderr}")


def topic_exists(topic: str) -> bool:
    return topic in ros2("topic", "list").stdout.splitlines()


def echo_once(topic: str, timeout_s: float = 15.0) -> str:
    """Return one message body from ``ros2 topic echo --once``."""
    r = ros2("topic", "echo", "--once", "--full-length", topic, timeout_s=timeout_s)
    if r.returncode != 0:
        raise RuntimeError(f"echo {topic} failed: {r.stderr}")
    return r.stdout


# --- main --------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(description="End-to-end ROS maps integration test")
    parser.add_argument("--ue-address", default="tcp://localhost",
                        help="URLab RPC address (default: tcp://localhost)")
    parser.add_argument("--scene", default=None,
                        help="Path to test MJCF; defaults to the bundled ros_maps_test.xml")
    parser.add_argument("--timeout", type=float, default=30.0,
                        help="Seconds to wait for each topic to appear")
    parser.add_argument("--keep-pie", action="store_true",
                        help="Leave PIE running after the test (default: stop PIE)")
    args = parser.parse_args()

    # Resolve the test scene.
    if args.scene:
        scene_path = Path(args.scene).resolve()
    else:
        scene_path = Path(__file__).resolve().parents[3] / "Content" / "TestData" / "ros_maps_test.xml"
    if not scene_path.exists():
        sys.exit(f"Test scene not found: {scene_path}")

    require_ros2()
    print(f"ROS 2 OK  | topics before test: {len(ros2('topic', 'list').stdout.splitlines())}")

    # --- Import the scene into UE ---------------------------------------
    from urlab_client import URLabClient

    client = URLabClient(args.ue_address, step_mode="direct")
    print("Connecting to UE...")
    client.connect(observations="standard")

    # Before PIE the map topics shouldn't exist (no manager = no providers).
    assert not topic_exists("/urlab/obstacle_cloud"), \
        "/urlab/obstacle_cloud already present before PIE — stale publishers?"

    print(f"Importing test scene: {scene_path}")
    bp = client.scene.import_xml(str(scene_path))
    print(f"import_xml → {bp.class_path}")

    # Spawn it into the level so the model is in the world at PIE start.
    client.scene.spawn_actor(bp, "ros_maps_test_actor")
    print("spawn_actor OK")

    # Start PIE.
    print("Starting PIE...")
    client.sim.start()
    assert client.manager_present, "PIE did not start (manager_present still False)"

    # Give providers a moment to build and publish.
    time.sleep(2.0)

    # --- Verify topics ---------------------------------------------------

    topics = {
        "/urlab/obstacle_cloud":  "PointCloud2",
        "/map":                   "OccupancyGrid",
        "/octomap_binary":        "Octomap",
    }

    for topic, msg_type in topics.items():
        deadline = time.monotonic() + args.timeout
        while not topic_exists(topic):
            if time.monotonic() > deadline:
                sys.exit(f"FAIL: {topic} ({msg_type}) did not appear within {args.timeout}s")
            print(f"  waiting for {topic}...")
            time.sleep(1.0)

        # Check topic type.
        info = ros2("topic", "info", topic)
        if msg_type not in info.stdout:
            sys.exit(f"FAIL: {topic} type mismatch — expected {msg_type}, got\n{info.stdout}")
        print(f"  {topic} ({msg_type}) — OK")

    # --- Content checks --------------------------------------------------

    # Point cloud: should have points.
    pc_echo = echo_once("/urlab/obstacle_cloud", timeout_s=15.0)
    if "height: 1" not in pc_echo and "is_dense: true" not in pc_echo:
        sys.exit(f"FAIL: /urlab/obstacle_cloud does not look like PointCloud2:\n{pc_echo[:500]}")
    # Count actual data lines — a non-empty cloud has 'data:' followed by hex or length.
    if "width:" in pc_echo:
        for line in pc_echo.splitlines():
            if "width:" in line.strip():
                width = int(line.strip().split()[-1])
                if width == 0:
                    sys.exit(f"FAIL: /urlab/obstacle_cloud has zero points: {line.strip()}")
                print(f"  /urlab/obstacle_cloud: {width} points — OK")
                break

    # Occupancy grid: latched, should have non-empty data.
    occ_echo = echo_once("/map", timeout_s=15.0)
    if "resolution:" not in occ_echo:
        sys.exit(f"FAIL: /map does not look like OccupancyGrid:\n{occ_echo[:500]}")
    has_occupied = False
    for line in occ_echo.splitlines():
        stripped = line.strip()
        if stripped.startswith("data:"):
            # The data array is printed as a list of int8 values.
            vals = stripped.removeprefix("data:").strip().strip("[]")
            if vals and any(v.strip() not in ("-1", "0") for v in vals.split(",")):
                has_occupied = True
    if has_occupied:
        print("  /map: occupied cells present — OK")
    else:
        print("  /map: no occupied cells found (may be outside grid bounds) — WARN")

    # Octomap: binary, check header.
    octo_echo = echo_once("/octomap_binary", timeout_s=15.0)
    if "resolution:" not in octo_echo and "binary: true" not in octo_echo:
        sys.exit(f"FAIL: /octomap_binary does not look like Octomap:\n{octo_echo[:500]}")
    print("  /octomap_binary: publishing — OK")

    # --- Cleanup ---------------------------------------------------------
    if not args.keep_pie:
        client.sim.stop()
        print("PIE stopped.")
    client.close()

    print("\n=== All ROS map providers verified ===")
    sys.exit(0)


if __name__ == "__main__":
    main()
