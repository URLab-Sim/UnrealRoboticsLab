#!/usr/bin/env python3
# Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
# Licensed under the Apache License, Version 2.0.
"""Auto-generate a MoveIt SRDF for a MuJoCo arm by *sampling actual collisions*.

This is the MoveIt Setup Assistant's disable-collisions algorithm, but the
collision truth comes from MuJoCo's own broadphase/narrowphase on the compiled
model (the same geometry the sim steps), so the disabled pairs match the physics
exactly instead of an approximate URDF mesh check.

For every ordered link pair we decide whether self-collision checking can be
disabled, and record the reason MoveIt uses:
  - Adjacent : linked by a joint (never a meaningful self-collision).
  - Default  : in collision in the model's default/home pose.
  - Always   : in collision in every sampled configuration (overlapping).
  - Never    : in collision in none of the sampled configurations.
Pairs that collide in *some* configurations keep collision checking on.

Groups and named states are derived from the model: the arm is the chain of
1-DoF joints from the base to the flange; named states come from <key>frames.

Usage:
  python generate_srdf.py --xml <model.xml> --out <robot.srdf> \
      [--samples 20000] [--arm-group panda_arm] [--seed 0]
"""
import argparse
import itertools
import sys
import xml.etree.ElementTree as ET

import numpy as np

try:
    import mujoco
except ImportError:
    sys.exit("mujoco is required (run under the bridge env: `uv run python ...`)")


def link_of_geom(m, gid):
    """URDF link (MuJoCo body) name a geom belongs to, or None for worldbody."""
    bid = m.geom_bodyid[gid]
    if bid == 0:
        return None
    return mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, bid)


def body_link_names(m):
    """All non-world body names, in id order (these are the URDF link names)."""
    return [
        mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b)
        for b in range(1, m.nbody)
    ]


def adjacent_pairs(m):
    """Link pairs connected directly by a joint (parent/child body)."""
    pairs = set()
    for b in range(1, m.nbody):
        p = m.body_parentid[b]
        if p == 0:
            continue
        a = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b)
        c = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, p)
        pairs.add(frozenset((a, c)))
    return pairs


def colliding_link_pairs(m, d):
    """Set of link pairs currently in contact (maps active contacts to links)."""
    out = set()
    for i in range(d.ncon):
        c = d.contact[i]
        la = link_of_geom(m, c.geom1)
        lb = link_of_geom(m, c.geom2)
        if la and lb and la != lb:
            out.add(frozenset((la, lb)))
    return out


def sample_qpos(m, rng):
    """Random qpos within joint ranges (limited joints) / [-pi, pi] (free hinges)."""
    q = m.qpos0.copy()
    for j in range(m.njnt):
        jtype = m.jnt_type[j]
        if jtype not in (mujoco.mjtJoint.mjJNT_HINGE, mujoco.mjtJoint.mjJNT_SLIDE):
            continue  # free/ball: leave at qpos0 (base stays put)
        adr = m.jnt_qposadr[j]
        if m.jnt_limited[j]:
            lo, hi = m.jnt_range[j]
        else:
            lo, hi = -np.pi, np.pi
        q[adr] = rng.uniform(lo, hi)
    return q


def keyframe_states(m):
    """{name: {joint_name: qpos}} for each <key>, 1-DoF joints only."""
    states = {}
    for k in range(m.nkey):
        name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_KEY, k) or f"key{k}"
        qpos = m.key_qpos[k]
        joints = {}
        for j in range(m.njnt):
            if m.jnt_type[j] not in (mujoco.mjtJoint.mjJNT_HINGE, mujoco.mjtJoint.mjJNT_SLIDE):
                continue
            jn = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j)
            joints[jn] = float(qpos[m.jnt_qposadr[j]])
        states[name] = joints
    return states


def arm_joint_chain(m):
    """1-DoF joint names from base to flange, in kinematic order (the arm)."""
    names = []
    for j in range(m.njnt):
        if m.jnt_type[j] != mujoco.mjtJoint.mjJNT_HINGE:
            continue
        names.append(mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j))
    return names


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--xml", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--robot-name", default="franka")
    ap.add_argument("--arm-group", default="panda_arm")
    ap.add_argument("--hand-group", default="hand")
    ap.add_argument("--hand-joints", default="finger_joint1,finger_joint2")
    ap.add_argument("--finger-open", type=float, default=0.04)
    ap.add_argument("--base-link", default="link0")
    ap.add_argument("--flange-link", default="hand")
    ap.add_argument("--samples", type=int, default=20000)
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()

    m = mujoco.MjModel.from_xml_path(args.xml)
    d = mujoco.MjData(m)
    rng = np.random.default_rng(args.seed)

    links = body_link_names(m)
    all_pairs = {frozenset(p) for p in itertools.combinations(links, 2)}
    adj = adjacent_pairs(m)

    # Default pose collisions.
    mujoco.mj_forward(m, d)
    default_col = colliding_link_pairs(m, d)

    # Sample: count how often each pair collides.
    ever = set()
    count = {p: 0 for p in all_pairs}
    for _ in range(args.samples):
        d.qpos[:] = sample_qpos(m, rng)
        mujoco.mj_forward(m, d)
        for p in colliding_link_pairs(m, d):
            if p in count:
                count[p] += 1
                ever.add(p)

    # Classify each pair -> (disabled?, reason).
    disabled = []  # (linkA, linkB, reason)
    for p in sorted(all_pairs, key=lambda s: sorted(s)):
        a, b = sorted(p)
        if p in adj:
            disabled.append((a, b, "Adjacent"))
        elif p in default_col:
            disabled.append((a, b, "Default"))
        elif count[p] >= args.samples:
            disabled.append((a, b, "Always"))
        elif count[p] == 0:
            disabled.append((a, b, "Never"))
        # else: sometimes collides -> keep checking

    # --- Emit SRDF ---
    robot = ET.Element("robot", name=args.robot_name)
    ET.SubElement(robot, "virtual_joint", name="virtual_joint", type="fixed",
                  parent_frame="world", child_link=args.base_link)

    arm = ET.SubElement(robot, "group", name=args.arm_group)
    ET.SubElement(arm, "chain", base_link=args.base_link, tip_link=args.flange_link)

    # Gripper group + end-effector. The finger joints are tendon-coupled in the
    # sim (one actuator drives both); MoveIt treats them as the hand group and the
    # gripper bridge maps the commanded opening onto that actuator.
    hand_joints = [j for j in args.hand_joints.split(",") if j]
    if hand_joints:
        hand = ET.SubElement(robot, "group", name=args.hand_group)
        for j in hand_joints:
            ET.SubElement(hand, "joint", name=j)
        ET.SubElement(robot, "end_effector", name="hand_ee",
                      parent_link=args.flange_link, group=args.hand_group,
                      parent_group=args.arm_group)
        for state, val in (("open", args.finger_open), ("closed", 0.0)):
            gs = ET.SubElement(robot, "group_state", name=state, group=args.hand_group)
            for j in hand_joints:
                ET.SubElement(gs, "joint", name=j, value=f"{val:.6g}")

    # Named states from keyframes (arm joints only).
    arm_joints = set(arm_joint_chain(m))
    for kname, joints in keyframe_states(m).items():
        gs = ET.SubElement(robot, "group_state", name=kname, group=args.arm_group)
        for jn, val in joints.items():
            if jn in arm_joints:
                ET.SubElement(gs, "joint", name=jn, value=f"{val:.6g}")

    for a, b, reason in disabled:
        ET.SubElement(robot, "disable_collisions", link1=a, link2=b, reason=reason)

    ET.indent(robot, space="  ")
    xml = '<?xml version="1.0"?>\n' + ET.tostring(robot, encoding="unicode")
    with open(args.out, "w", encoding="utf-8") as f:
        f.write(xml + "\n")

    kept = len(all_pairs) - len(disabled)
    by_reason = {}
    for _, _, r in disabled:
        by_reason[r] = by_reason.get(r, 0) + 1
    print(f"links: {len(links)}  pairs: {len(all_pairs)}")
    print(f"disabled: {len(disabled)}  {by_reason}")
    print(f"kept (self-collision checked): {kept}")
    print(f"wrote {args.out}")


if __name__ == "__main__":
    main()
