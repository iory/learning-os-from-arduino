#!/usr/bin/env python
"""Build the arduino_os_quad_robot MJCF: normalised URDF -> mjcf_converter -> RL bits.

Pipeline (all of it reproducible from the SolidWorks export):

  1. ``normalize_urdf.py``   CAD names/axis signs -> FL/FR/RL/RR + one sign
                             convention (run this first; not called from here so
                             the generated URDF stays reviewable).
  2. ``skrobot.urdf.urdf_to_mjcf``  floating base, meshes -> STL, position
                             actuators with the URDF's effort as forcerange.
  3. this file's post-process:
       * a named contact sphere at each toe (``FL_toe`` ...)
       * a site at each toe (foot clearance / slip rewards, ``FL`` ...)
       * an IMU site on the base + gyro / velocimeter / accelerometer /
         subtree-angular-momentum sensors, named the way the mjlab velocity task
         expects (``imu_ang_vel``, ``imu_lin_vel``, ``root_angmom``)
       * the home keyframe

Unlike the larger 5 DoF leg project there is **no mass-reconstruction stage**: the CAD
export carries per-link SolidWorks masses that sum to 1.0378 kg, which is a
usable number for a 1 kg servo robot, so we keep them.

Physics constants below are STS3215 (12 V variant) datasheet values, or estimates
clearly marked as such. The estimates are the system-identification list for
sim2real -- see docs/arduino_os_quad_robot.md.

Run from the repo root::

  uv run --with 'scikit-robot>=0.3.21' --with mujoco \
      python arduino_os_quad_robot/scripts/build_mjcf.py
"""
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROBOT = os.path.dirname(HERE)

from skrobot.urdf import urdf_to_mjcf  # noqa: E402

# Overridable so a new CAD export can be built without overwriting the model the
# shipped policy was trained on. Same reason as normalize_urdf.py.
#   build_mjcf.py [SRC_rl.urdf] [DST.xml]
URDF = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
    ROBOT, "urdf", "arduino_os_quad_robot_rl.urdf")
MJCF = sys.argv[2] if len(sys.argv) > 2 else os.path.join(
    ROBOT, "mjcf", "arduino_os_quad_robot.xml")

LEGS = ("FL", "FR", "RL", "RR")

##
# Actuator / joint physics. Feetech STS3215/ST3215 serial bus servo, 12 V
# variant (1:345 gearbox, 4096 counts/turn):
#
#   stall torque   30 kgf.cm = 2.942 Nm    -> already the URDF <limit effort>
#   no-load speed  45 rpm    = 4.717 rad/s -> already the URDF <limit velocity>
#   rated torque   10 kgf.cm = 0.98 Nm     (continuous; not modelled, no thermals)
#
# The URDF limits match this variant exactly. The 7.4 V STS3215 (C001) is a
# different motor -- 19.5 kgf.cm = 1.91 Nm stall -- so check which one is
# actually bolted to the robot before trusting any of this.
#
# A DC motor's available torque falls linearly from stall to zero at no-load
# speed. Rather than a special actuator type, that slope is modelled as passive
# viscous damping on the joint:
#
#   b_backemf = tau_stall / omega_noload = 2.942 / 4.717 = 0.6237 Nm.s/rad
#
# so a joint driven at full torque coasts out at exactly the datasheet no-load
# speed. Leaving this out is the classic servo-robot sim2real failure: the sim
# swings the legs far faster than a 45 rpm servo ever can.
##
BACKEMF_DAMPING = 2.942 / 4.717  # 0.6237 Nm.s/rad, from the datasheet pair

# ESTIMATE (system-ID item #1). Reflected rotor inertia of a 1:345 gearbox:
# J_rotor ~ 2e-8 kg.m^2 (few-gram rotor, 6 mm dia) * 345^2 = 2.4e-3.
JOINT_ARMATURE = 0.002

# ESTIMATE (system-ID item #2). Effective P gain of the servo's internal loop:
# 2.942 Nm of drive at ~0.15 rad of error.
ACTUATOR_KP = 20.0

# Home stance, all four legs identical, knee apex pointing FORWARD. Solved so
# each toe sits directly under its own hip axis with the base 0.14 m up; the
# straight-leg maximum is 0.190 m.
# Self-interference was scanned over hip in +-1.6 / knee in +-2.0 rad: the
# closest angle-dependent pair stays 23 mm apart, so nothing here is mechanically
# blocked (servo horn range and cable routing are not modelled).
# Knee TRAILING (see mjlab_overlay/arduino_os_quad_robot/robot_cfg.py). Angles
# are in the yaw-flipped frame normalize_urdf.py emits.
HOME_HIP = 0.7748
HOME_KNEE = -1.3474
HOME_BASE_HEIGHT = 0.14
HOME = {}
for _leg in LEGS:
    HOME["{}_hip_joint".format(_leg)] = HOME_HIP
    HOME["{}_knee_joint".format(_leg)] = HOME_KNEE

# Toe contact patch, measured on bracket_step__bracket_1_step_1.glb: the 13x12 mm
# end face of the bracket, centroid at this point in the foot link frame (same
# for all four legs -- they share the mesh).
TOE_LOCAL = np.array([-0.0899, 0.002, 0.0744])
TOE_RADIUS = 0.008  # contact sphere; the real end face is ~13 mm across

BASE_BODY = "base_link"


def _rpy_to_mat(r, p, y):
    cr, sr, cp, sp, cy, sy = (
        np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y))
    return (np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
            @ np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
            @ np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]]))


def foot_frames():
    """Rotation of each ``<leg>_foot`` link at q=0, from the normalised URDF.

    Needed to push the contact sphere's centre TOE_RADIUS *up* (world +Z) from
    the measured toe point, so the sphere is tangent to the real end face
    instead of hovering above it. The left and right feet are mirrored, so the
    local "up" direction differs per leg.
    """
    root = ET.parse(URDF).getroot()
    by_child = {j.find("child").get("link"): j for j in root.findall("joint")}
    frames = {}
    for leg in LEGS:
        link = "{}_foot".format(leg)
        R, chain = np.eye(3), []
        node = link
        while node != BASE_BODY:
            j = by_child[node]
            chain.append(j)
            node = j.find("parent").get("link")
        for j in reversed(chain):
            rpy = [float(v) for v in j.find("origin").get("rpy").split()]
            R = R @ _rpy_to_mat(*rpy)
        frames[leg] = R
    return frames


def add_rl_sites_and_geoms(mjcf_path):
    """Toe contact spheres + toe sites + IMU site and sensors."""
    tree = ET.parse(mjcf_path)
    root = tree.getroot()
    bodies = {b.get("name"): b for b in root.iter("body")}
    frames = foot_frames()

    for leg in LEGS:
        body = bodies["{}_foot".format(leg)]
        # world +Z expressed in this foot link's frame at q=0
        up_local = frames[leg].T @ np.array([0.0, 0.0, 1.0])
        centre = TOE_LOCAL + TOE_RADIUS * up_local
        geom_name = "{}_toe".format(leg)  # NOT *_foot_collision: the mesh geom already owns that name
        if not any(g.get("name") == geom_name for g in body.findall("geom")):
            ET.SubElement(body, "geom", {
                "name": geom_name,
                "type": "sphere",
                "size": "{:g}".format(TOE_RADIUS),
                "pos": "{:.6g} {:.6g} {:.6g}".format(*centre),
                "contype": "2", "conaffinity": "1",
                "rgba": "0.1 0.1 0.1 1",
            })
        if not any(s.get("name") == leg for s in body.findall("site")):
            ET.SubElement(body, "site", {
                "name": leg,
                "pos": "{:.6g} {:.6g} {:.6g}".format(*TOE_LOCAL),
                "size": "0.005", "group": "4", "rgba": "1 0 0 1",
            })

    base = bodies[BASE_BODY]
    if not any(s.get("name") == "imu_in_body" for s in base.findall("site")):
        ET.SubElement(base, "site", {
            "name": "imu_in_body", "pos": "0 0 0",
            "size": "0.005", "group": "4", "rgba": "0 1 0 1",
        })

    # The task's critic reads these by name; the actor does not (this robot has
    # no IMU -- see mjlab_overlay/arduino_os_quad_robot/env_cfgs.py).
    if root.find("sensor") is None:
        sensor = ET.SubElement(root, "sensor")
        ET.SubElement(sensor, "gyro", {"name": "imu_ang_vel", "site": "imu_in_body"})
        ET.SubElement(sensor, "velocimeter",
                      {"name": "imu_lin_vel", "site": "imu_in_body"})
        ET.SubElement(sensor, "accelerometer",
                      {"name": "imu_lin_acc", "site": "imu_in_body"})
        ET.SubElement(sensor, "subtreeangmom",
                      {"name": "root_angmom", "body": BASE_BODY})

    ET.indent(tree, space="  ")
    tree.write(mjcf_path, encoding="unicode", xml_declaration=False)


def main():
    if not os.path.exists(URDF):
        raise SystemExit(
            "{} is missing -- run arduino_os_quad_robot/scripts/normalize_urdf.py "
            "first.".format(URDF))
    os.makedirs(os.path.dirname(MJCF), exist_ok=True)
    urdf_to_mjcf(
        URDF, MJCF,
        floating_base=True,
        self_collision=False,
        add_position_actuators=True,
        actuator_kp=ACTUATOR_KP,
        add_actuator_forcerange=True,
        joint_armature=JOINT_ARMATURE,
        joint_damping=BACKEMF_DAMPING,
        add_ground=True,
        home=HOME,
        home_base_height=HOME_BASE_HEIGHT,
    )
    print("stage 1: converted URDF -> {}".format(MJCF))
    add_rl_sites_and_geoms(MJCF)
    print("stage 2: toe spheres (r={:g} m) + toe sites + IMU site/sensors added"
          .format(TOE_RADIUS))
    return 0


if __name__ == "__main__":
    sys.exit(main())
