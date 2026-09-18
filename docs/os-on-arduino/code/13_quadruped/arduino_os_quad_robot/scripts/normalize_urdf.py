#!/usr/bin/env python
"""Normalise the SolidWorks-exported quadruped URDF for RL / deployment use.

The exporter names links after assembly instances
(``Assem130_right_6__bracket_step_1__bracket_1_step_1``) and keeps whatever
joint axis direction the CAD mates produced.  Two things make that unusable
downstream:

1. **Names.**  Nothing in the name says which leg it is, so every task config,
   reward regex and servo-ID table would have to carry a lookup table.
2. **Axis signs.**  The hip axes are mirrored left/right (world ``+Y`` on the
   ``y<0`` legs, ``-Y`` on the ``y>0`` legs) but the knee axes are *not* (all
   four are world ``+Y``).  Commanding the same angle on all four legs
   therefore bends two legs one way and two the other -- measured: a uniform
   ``hip=-0.3, knee=+0.6`` tips the robot 55.9 deg.

This script rewrites both.  Geometry is untouched: renaming is cosmetic and an
axis flip is exactly compensated by negating that joint's angle, so the
normalised URDF describes the same machine.  On the real robot each servo's
positive direction is a calibration constant anyway (see the sign table printed
at the end, which is what the firmware needs).

Leg naming (front = +X, ROS convention +Y = left), from the measured hip
positions in the exported URDF::

    exporter prefix     hip (x, y) in CAD   after yaw flip     canonical
    Assem130_right_6    (-0.090, +0.052)    (+0.090, -0.052)   FR  front right
    Assem130_5          (-0.090, -0.052)    (+0.090, +0.052)   FL  front left
    Assem130_right_5    (+0.060, +0.052)    (-0.060, -0.052)   RR  rear  right
    Assem130_6          (+0.060, -0.052)    (-0.060, +0.052)   RL  rear  left

The whole robot is yaw-flipped 180 deg first (see ``yaw_flip``): the CAD's +X end
measured as the worse leading end by a factor of nearly two in speed, so it is
the tail.

Canonical sign convention after normalisation (all four legs identical, so a
trot is a phase shift and nothing else):

* ``<leg>_hip_joint``  axis = world +Y, positive -> toe swings BACKWARD (-X)
* ``<leg>_knee_joint`` axis = world +Y, positive -> toe swings BACKWARD (-X)

Usage::

    uv run --with scikit-robot python arduino_os_quad_robot/scripts/normalize_urdf.py

Reads  ``urdf/arduino_os_quad_robot.urdf``     (SolidWorks export, never edited)
Writes ``urdf/arduino_os_quad_robot_rl.urdf``  (generated, committed)
"""
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np

ROBOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
# Paths default to the in-tree robot but can be overridden, so a new CAD export
# can be normalised without clobbering the one the shipped policy was built from.
#   normalize_urdf.py [SRC.urdf] [DST.urdf]
# They used to be constants while the script silently accepted (and ignored)
# argv, which quietly re-normalised the old robot when handed a new one.
SRC = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
    ROBOT, "urdf", "arduino_os_quad_robot.urdf")
DST = sys.argv[2] if len(sys.argv) > 2 else os.path.join(
    ROBOT, "urdf", "arduino_os_quad_robot_rl.urdf")

# exporter prefix -> canonical leg tag.  Verified against the hip positions
# recomputed below; the script aborts if the CAD export moves a leg.
# Leg names are assigned AFTER the 180 deg yaw below, so these are the rotated
# coordinates. The CAD export's +X end is the TAIL: see the module docstring.
LEG_MAP = {
    "Assem130_right_6": "FR",
    "Assem130_5": "FL",
    "Assem130_right_5": "RR",
    "Assem130_6": "RL",
}
# Measured from the 2026-08-22 CAD export.  The pre-2026-08 export had the same
# x but y = +-0.0517; it is 1.1 mm outside HIP_XY_TOL, so this table also acts as
# a check that the intended CAD revision is the one being normalised.
EXPECTED_HIP_XY = {  # metres, world frame at q=0
    "FR": (+0.090, -0.0528),
    "FL": (+0.090, +0.0528),
    "RR": (-0.060, -0.0528),
    "RL": (-0.060, +0.0528),
}
HIP_XY_TOL = 2e-3

# exporter link suffix -> canonical link suffix
LINK_SUFFIX = {
    "STS3215_03a_nopassivehorn_1": "hip_servo",
    "leg_link1_2": "thigh",
    "STS3215_03a_nopassivehorn_3": "knee_servo",
    "bracket_step_1__bracket_1_step_1": "foot",   # 2026-08 以前の CAD
    "bracket_outline_1": "foot",                  # 2026-08-22 以降の CAD
}
# canonical child link suffix -> canonical joint name suffix
JOINT_FOR_CHILD = {
    "hip_servo": "hip_mount",   # fixed: body -> hip servo case
    "thigh": "hip_joint",       # revolute: hip pitch
    "knee_servo": "knee_mount",  # fixed: thigh -> knee servo case
    "foot": "knee_joint",       # revolute: knee pitch
}


def rpy_to_mat(r, p, y):
    cr, sr, cp, sp, cy, sy = (
        np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y))
    return (np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
            @ np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
            @ np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]]))


def canonical_link_name(name):
    """``Assem130_right_6__bracket_step_1__bracket_1_step_1`` -> ``RL_foot``."""
    if name == "base_link":
        return name
    prefix, _, suffix = name.partition("__")
    if prefix not in LEG_MAP or suffix not in LINK_SUFFIX:
        raise KeyError("unmapped link {!r} (prefix={!r} suffix={!r})".format(
            name, prefix, suffix))
    return "{}_{}".format(LEG_MAP[prefix], LINK_SUFFIX[suffix])


def yaw_flip(root):
    """Rotate everything under base_link 180 deg about Z, so the task's forward
    (+X) is the end the robot should actually lead with.

    The CAD export puts the four hips at x = +0.060 and x = -0.090 and the body's
    solid section at +X. Two things had to be reconciled: the physical build has
    the knee apex pointing toward that solid section, and a scripted trot walks
    the machine far better with the knee apex trailing. Measured on the MJCF
    (arduino_os_quad_robot/scripts/openloop_trot.py), same assembly both ways:

        leading with the solid end   0.089 m/s, body pitch up to 21.5 deg
        leading with the other end   0.156 m/s, body pitch 8.3 deg

    So the solid end is the tail. Rotating here rather than negating commands
    later keeps every downstream convention (task forward = base +X, ROS +Y =
    left) intact, and leaves joint-space home poses untouched.

    Note the machine is NOT fore-aft symmetric even though the four legs are
    identical parts: every hip servo puts its horn 11.9 mm toward the CAD's -X,
    so this flip is a real change, not a relabelling.
    """
    R = np.array([[-1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0]])  # Rz(180)
    rpy_z = np.array([0.0, 0.0, np.pi])

    def rot_origin(el):
        o = el.find("origin")
        if o is None:
            return
        xyz = np.array([float(v) for v in (o.get("xyz") or "0 0 0").split()])
        rpy = np.array([float(v) for v in (o.get("rpy") or "0 0 0").split()])
        o.set("xyz", "{:.10g} {:.10g} {:.10g}".format(*(R @ xyz)))
        M = rpy_to_mat(*rpy_z) @ rpy_to_mat(*rpy)
        # back to rpy (ZYX)
        sy = -M[2, 0]
        pitch = np.arcsin(np.clip(sy, -1.0, 1.0))
        roll = np.arctan2(M[2, 1], M[2, 2])
        yaw = np.arctan2(M[1, 0], M[0, 0])
        o.set("rpy", "{:.10g} {:.10g} {:.10g}".format(roll, pitch, yaw))

    for j in root.findall("joint"):
        if j.find("parent").get("link") == "base_link":
            rot_origin(j)
    base = [l for l in root.findall("link") if l.get("name") == "base_link"][0]
    for tag in ("visual", "collision", "inertial"):
        for el in base.findall(tag):
            rot_origin(el)


def main():
    tree = ET.parse(SRC)
    root = tree.getroot()
    root.set("name", "arduino_os_quad_robot")
    # The 2026-08-22 CAD export already faces the right way, so the default is
    # no flip.  $QUAD_YAW_FLIP=1 re-enables it for the pre-2026-08 export,
    # which came out tail-first.  Applying the flip to an already-flipped
    # export turns the robot back round, and nothing downstream notices -- it
    # just walks the wrong end first.
    if os.environ.get("QUAD_YAW_FLIP", "0") == "1":
        yaw_flip(root)
        print("yaw flip: applied (QUAD_YAW_FLIP=1, pre-2026-08 CAD)")

    joints = root.findall("joint")
    by_child = {j.find("child").get("link"): j for j in joints}

    # --- forward kinematics at q=0, to get each joint's world axis ---------
    def world_frame(link):
        R, p, chain = np.eye(3), np.zeros(3), []
        while link != "base_link":
            j = by_child[link]
            chain.append(j)
            link = j.find("parent").get("link")
        for j in reversed(chain):
            o = j.find("origin")
            xyz = np.array([float(v) for v in o.get("xyz").split()])
            rpy = np.array([float(v) for v in o.get("rpy").split()])
            p = p + R @ xyz
            R = R @ rpy_to_mat(*rpy)
        return R, p

    flips, sign_table = [], []
    for j in joints:
        if j.get("type") != "revolute":
            continue
        child = j.find("child").get("link")
        R, p = world_frame(child)
        axis_el = j.find("axis")
        axis = np.array([float(v) for v in axis_el.get("xyz").split()])
        world_axis = R @ axis
        name = canonical_link_name(child)
        leg, suffix = name.split("_", 1)
        # Canonical: world axis = +Y for every revolute joint.
        flip = world_axis[1] < 0.0
        if flip:
            axis_el.set("xyz", "{:g} {:g} {:g}".format(*(-axis)))
            flips.append(name)
        sign_table.append((leg, JOINT_FOR_CHILD[suffix], -1 if flip else +1,
                           p, R @ (-axis if flip else axis)))

    # --- check the leg map still matches the CAD export --------------------
    for j in joints:
        child = j.find("child").get("link")
        if not child.endswith("__leg_link1_2"):
            continue
        leg = canonical_link_name(child).split("_")[0]
        _, p = world_frame(child)
        ex, ey = EXPECTED_HIP_XY[leg]
        if abs(p[0] - ex) > HIP_XY_TOL or abs(p[1] - ey) > HIP_XY_TOL:
            raise SystemExit(
                "leg map is stale: {} hip is at ({:.4f}, {:.4f}) but LEG_MAP "
                "expects ({:.4f}, {:.4f}). Re-derive LEG_MAP from the new CAD "
                "export before regenerating.".format(leg, p[0], p[1], ex, ey))

    # --- rename links and joints ------------------------------------------
    for link in root.findall("link"):
        link.set("name", canonical_link_name(link.get("name")))
    for j in joints:
        child = canonical_link_name(j.find("child").get("link"))
        parent = canonical_link_name(j.find("parent").get("link"))
        j.find("child").set("link", child)
        j.find("parent").set("link", parent)
        leg, suffix = child.split("_", 1)
        j.set("name", "{}_{}".format(leg, JOINT_FOR_CHILD[suffix]))

    ET.indent(tree, space="  ")
    with open(DST, "w") as f:
        f.write('<?xml version="1.0"?>\n')
        f.write("<!-- GENERATED by arduino_os_quad_robot/scripts/normalize_urdf.py "
                "from arduino_os_quad_robot.urdf. Do not edit by hand. -->\n")
        tree.write(f, encoding="unicode")
        f.write("\n")

    print("wrote {}".format(DST))
    print("flipped axis on {} joints: {}".format(
        len(flips), ", ".join(sorted(flips)) or "(none)"))
    print()
    print("canonical joints (world frame at q=0; sign = normalised vs CAD export):")
    print("  {:16s} {:>5s}  {:>26s}  {:>20s}".format(
        "joint", "sign", "axis position (x y z)", "world axis"))
    for leg, suffix, sign, p, ax in sorted(sign_table):
        print("  {:16s} {:+5d}  {:8.4f} {:8.4f} {:8.4f}  {:6.2f} {:6.2f} {:6.2f}".format(
            "{}_{}".format(leg, suffix), sign, p[0], p[1], p[2], ax[0], ax[1], ax[2]))
    print()
    print("firmware note: 'sign' is the factor between the normalised joint angle "
          "(what the policy outputs) and this URDF's CAD-native direction. The "
          "servo-side sign is a separate calibration constant per servo.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
