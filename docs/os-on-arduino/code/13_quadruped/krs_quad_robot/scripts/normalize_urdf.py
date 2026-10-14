#!/usr/bin/env python
"""Normalise the SolidWorks-exported KRS quadruped URDF for RL / deployment.

The KRS version counterpart of ``arduino_os_quad_robot/scripts/normalize_urdf.py``.
The sw2robot export (``urdf/krs_quad_robot.urdf``, never edited) has 90 links --
every screw, horn and servo case is its own fixed link -- named after assembly
instances, with the axis directions the CAD mates produced and the masses
SolidWorks computed from material density. This script writes
``urdf/krs_quad_robot_rl.urdf`` with:

1. **Nine links.** Every fixed subtree is merged into the moving body it rides
   on: ``base_link``, ``<leg>_thigh`` (leg link + knee servo, driven by the hip)
   and ``<leg>_foot`` (the bracket, driven by the knee). Visual and collision
   meshes are kept, re-expressed in the new frames; inertias are combined with
   the parallel-axis theorem.
2. **Front = +X.** The CAD's base_link has the front at -X and the right side at
   +Y. Everything under base_link is turned 180 deg about Z, so the task's
   forward is +X and +Y is left (ROS convention), the same as the Feetech
   version.
3. **Names and axis signs.** Joints become ``<leg>_hip_joint`` /
   ``<leg>_knee_joint`` for leg in FL/FR/RL/RR, and every axis is world +Y, so
   a positive angle swings the toe BACKWARD on all eight joints (the convention
   ``include/quad_calib_rcb4.h`` is calibrated against). q = 0 stays the CAD
   pose: legs straight down, servo centre (pulse 7500).
4. **Real masses.** Each CAD part's mass is replaced by the measured one
   (README.md, "質量"); its inertia is scaled by the same factor, i.e. the CAD
   density distribution is kept. The battery, RCB-4mini and Arduino, which are
   not in the CAD, are added to base_link as solid boxes.

Usage (from code/13_quadruped)::

    uv run --no-project --with numpy python krs_quad_robot/scripts/normalize_urdf.py
"""
import os
import sys
import xml.etree.ElementTree as ET
from collections import defaultdict

import numpy as np

ROBOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
    ROBOT, "urdf", "krs_quad_robot.urdf")
DST = sys.argv[2] if len(sys.argv) > 2 else os.path.join(
    ROBOT, "urdf", "krs_quad_robot_rl.urdf")
PACKAGE = "krs_quad_robot"

# Hip servo instance in the CAD -> leg, from README.md's servo-ID table
# (KRS3300_3 = right front = ID 1, ...). Named in the turned (front = +X) frame.
HIP_SERVO_LEG = {
    "KRS3300_3": "FR",
    "KRS3300_4": "FL",
    "KRS3300_1": "RR",
    "KRS3300_2": "RL",
}
# Checked against the hip axis positions after the turn; the script aborts if
# the CAD export has moved a leg.
EXPECTED_HIP_XY = {  # m, turned frame, q = 0
    "FR": (+0.066, -0.0306),
    "FL": (+0.066, +0.0306),
    "RR": (-0.066, -0.0306),
    "RL": (-0.066, +0.0306),
}
HIP_XY_TOL = 2e-3

# Output link order. It decides the joint order in the MJCF and therefore the
# order of the policy's 8 outputs; this is the order quad_calib_rcb4.h uses.
LEG_ORDER = ("FL", "RL", "RR", "FR")

##
# Masses (README.md). Matched by substring of the CAD link name; the first
# match wins. Parts not listed (screws, horns: 5.6 g in all) keep the CAD mass.
##
MASS_OVERRIDE = (
    ("KRS3300_scale", 0.0264),        # KRS-3302 ICS, kondo-robot.com/product/03146
    ("bracket_outline_krs", 0.0109),  # slicer, 43.4 g / 4
    ("leg_link1_krs", 0.0183),        # slicer, 73.22 g / 4
    ("lid_krs", 0.0413),              # slicer
)
BODY_MASS = 0.1253                    # body_krs (base_link's own mesh), slicer

# Parts that are not in the CAD, as solid boxes in the CAD's base_link frame
# (front = -X, right = +Y). centre [m], size [m], mass [kg].
EXTRA_PARTS = (
    # ROBO Power Cell E 6N-800, flat on the floor (z = 3 mm) under the Arduino.
    # Position is an ESTIMATE (README: 目安); the long side is assumed along X.
    ("battery", (0.004, 0.021, 0.014), (0.048, 0.0325, 0.022), 0.078),
    # Arduino UNO R4 WiFi. 25 g is an ASSUMPTION (no official figure; UNO R3's).
    # Board outline from README; 15 mm tall with the headers.
    ("arduino", (0.0038, 0.0214, 0.0375), (0.0685, 0.0533, 0.015), 0.025),
    # RCB-4mini, 35 x 30 x 12 mm, kondo-robot.com/product/rcb4_mini.
    ("rcb4mini", (0.023, -0.0225, 0.032), (0.035, 0.030, 0.012), 0.0076),
)

TURN = np.diag([-1.0, -1.0, 1.0])  # Rz(180 deg): CAD base frame -> turned frame


def rpy_to_mat(r, p, y):
    cr, sr, cp, sp, cy, sy = (
        np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y))
    return (np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
            @ np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
            @ np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]]))


def mat_to_rpy(m):
    pitch = np.arcsin(np.clip(-m[2, 0], -1.0, 1.0))
    roll = np.arctan2(m[2, 1], m[2, 2])
    yaw = np.arctan2(m[1, 0], m[0, 0])
    return roll, pitch, yaw


def read_origin(el):
    o = el.find("origin")
    if o is None:
        return np.eye(3), np.zeros(3)
    xyz = np.array([float(v) for v in (o.get("xyz") or "0 0 0").split()])
    rpy = [float(v) for v in (o.get("rpy") or "0 0 0").split()]
    return rpy_to_mat(*rpy), xyz


def fmt(v):
    return " ".join("{:.10g}".format(float(x)) for x in v)


def cad_mass(name, mass):
    for key, m in MASS_OVERRIDE:
        if key in name:
            return m
    if name == "base_link":
        return BODY_MASS
    return mass


def box_inertia(m, size):
    a, b, c = size
    return np.diag([m * (b * b + c * c), m * (a * a + c * c), m * (a * a + b * b)]) / 12.0


def main():
    root = ET.parse(SRC).getroot()
    links = {l.get("name"): l for l in root.findall("link")}
    joints = root.findall("joint")
    by_child = {j.find("child").get("link"): j for j in joints}
    kids = defaultdict(list)
    for j in joints:
        kids[j.find("parent").get("link")].append(j)

    # --- forward kinematics at q = 0, in the turned frame --------------------
    world = {"base_link": (TURN, np.zeros(3))}
    stack = ["base_link"]
    while stack:
        p = stack.pop()
        for j in kids[p]:
            c = j.find("child").get("link")
            R, t = read_origin(j)
            Rp, tp = world[p]
            world[c] = (Rp @ R, tp + Rp @ t)
            stack.append(c)

    # --- which revolute joint is which ---------------------------------------
    revolute = [j for j in joints if j.get("type") == "revolute"]

    def owner(link):
        """The revolute joint that moves ``link`` (None = base_link)."""
        while link != "base_link":
            j = by_child[link]
            if j.get("type") == "revolute":
                return j
            link = j.find("parent").get("link")
        return None

    role = {}  # revolute joint name -> (leg, "hip" | "knee")
    for j in revolute:
        if owner(j.find("parent").get("link")) is None:
            servo = [k for k in HIP_SERVO_LEG if "__{}__".format(k) in j.get("name")]
            assert len(servo) == 1, j.get("name")
            role[j.get("name")] = (HIP_SERVO_LEG[servo[0]], "hip")
    for j in revolute:
        if j.get("name") not in role:
            hip = owner(j.find("parent").get("link"))
            role[j.get("name")] = (role[hip.get("name")][0], "knee")
    assert sorted(role.values()) == sorted(
        (leg, r) for leg in LEG_ORDER for r in ("hip", "knee")), role

    def body_of(link):
        j = owner(link)
        if j is None:
            return "base_link"
        leg, r = role[j.get("name")]
        return "{}_{}".format(leg, "thigh" if r == "hip" else "foot")

    # --- new link frames: at the joint axis, axes parallel to the turned base --
    frame = {"base_link": (np.eye(3), np.zeros(3))}
    joint_info = {}
    for j in revolute:
        leg, r = role[j.get("name")]
        child = j.find("child").get("link")
        R, p = world[child]
        axis = R @ np.array([float(v) for v in j.find("axis").get("xyz").split()])
        assert abs(abs(axis[1]) - 1.0) < 1e-6, (j.get("name"), axis)
        body = "{}_{}".format(leg, "thigh" if r == "hip" else "foot")
        frame[body] = (np.eye(3), p)
        lim = j.find("limit")
        joint_info[body] = dict(
            name="{}_{}_joint".format(leg, r),
            parent=body_of(j.find("parent").get("link")),
            sign=1 if axis[1] > 0 else -1, pos=p,
            lower=float(lim.get("lower")), upper=float(lim.get("upper")),
            effort=lim.get("effort"), velocity=lim.get("velocity"))

    for leg, (ex, ey) in EXPECTED_HIP_XY.items():
        p = frame["{}_thigh".format(leg)][1]
        if abs(p[0] - ex) > HIP_XY_TOL or abs(p[1] - ey) > HIP_XY_TOL:
            raise SystemExit(
                "{} hip is at ({:.4f}, {:.4f}), expected ({:.4f}, {:.4f}): the CAD "
                "export changed, re-check HIP_SERVO_LEG".format(leg, p[0], p[1], ex, ey))

    # --- merge geometry and inertia into the nine bodies ---------------------
    visuals, collisions = defaultdict(list), defaultdict(list)
    inertia_parts = defaultdict(list)  # body -> [(m, com, I) in body frame]
    for name, link in links.items():
        body = body_of(name)
        Rw, pw = world[name]
        Rb, pb = frame[body]
        # this link's frame expressed in the body frame
        R_lb = Rb.T @ Rw
        p_lb = Rb.T @ (pw - pb)
        for tag, out in (("visual", visuals), ("collision", collisions)):
            for el in link.findall(tag):
                R, t = read_origin(el)
                out[body].append((R_lb @ R, p_lb + R_lb @ t,
                                  el.find("geometry/mesh").get("filename")))
        inert = link.find("inertial")
        if inert is None:
            continue
        m_cad = float(inert.find("mass").get("value"))
        m = cad_mass(name, m_cad)
        R, t = read_origin(inert)
        i = inert.find("inertia")
        I = np.array([[float(i.get("ixx")), float(i.get("ixy")), float(i.get("ixz"))],
                      [float(i.get("ixy")), float(i.get("iyy")), float(i.get("iyz"))],
                      [float(i.get("ixz")), float(i.get("iyz")), float(i.get("izz"))]])
        R_ib = R_lb @ R
        inertia_parts[body].append(
            (m, p_lb + R_lb @ t, R_ib @ (I * (m / m_cad)) @ R_ib.T))

    for name, centre, size, m in EXTRA_PARTS:
        inertia_parts["base_link"].append(
            (m, TURN @ np.array(centre), box_inertia(m, size)))

    def combine(parts):
        m = sum(p[0] for p in parts)
        com = sum(p[0] * p[1] for p in parts) / m
        I = np.zeros((3, 3))
        for mi, ci, Ii in parts:
            d = ci - com
            I += Ii + mi * (d @ d * np.eye(3) - np.outer(d, d))
        return m, com, I

    # --- write -----------------------------------------------------------------
    out = ET.Element("robot", {"name": "krs_quad_robot"})
    bodies = ["base_link"] + ["{}_{}".format(leg, s)
                              for leg in LEG_ORDER for s in ("thigh", "foot")]
    report = []
    for body in bodies:
        link = ET.SubElement(out, "link", {"name": body})
        m, com, I = combine(inertia_parts[body])
        report.append((body, m, com))
        inert = ET.SubElement(link, "inertial")
        ET.SubElement(inert, "origin", {"xyz": fmt(com), "rpy": "0 0 0"})
        ET.SubElement(inert, "mass", {"value": "{:.6g}".format(m)})
        ET.SubElement(inert, "inertia", {
            "ixx": "{:.6g}".format(I[0, 0]), "ixy": "{:.6g}".format(I[0, 1]),
            "ixz": "{:.6g}".format(I[0, 2]), "iyy": "{:.6g}".format(I[1, 1]),
            "iyz": "{:.6g}".format(I[1, 2]), "izz": "{:.6g}".format(I[2, 2])})
        for tag, items in (("visual", visuals[body]), ("collision", collisions[body])):
            for R, t, fn in items:
                el = ET.SubElement(link, tag)
                ET.SubElement(el, "origin", {"xyz": fmt(t), "rpy": fmt(mat_to_rpy(R))})
                g = ET.SubElement(el, "geometry")
                ET.SubElement(g, "mesh", {
                    "filename": "package://{}/meshes/{}".format(
                        PACKAGE, fn.split("/")[-1])})
        if body == "base_link":
            continue
        info = joint_info[body]
        j = ET.SubElement(out, "joint", {"name": info["name"], "type": "revolute"})
        ET.SubElement(j, "origin", {
            "xyz": fmt(info["pos"] - frame[info["parent"]][1]), "rpy": "0 0 0"})
        ET.SubElement(j, "parent", {"link": info["parent"]})
        ET.SubElement(j, "child", {"link": body})
        ET.SubElement(j, "axis", {"xyz": "0 1 0"})
        # Flipping the axis negates the angle; the +-135 deg range is symmetric.
        lo, hi = sorted((info["sign"] * info["lower"], info["sign"] * info["upper"]))
        ET.SubElement(j, "limit", {"lower": "{:.6g}".format(lo),
                                   "upper": "{:.6g}".format(hi),
                                   "effort": info["effort"],
                                   "velocity": info["velocity"]})

    tree = ET.ElementTree(out)
    ET.indent(tree, space="  ")
    with open(DST, "w") as f:
        f.write('<?xml version="1.0"?>\n')
        f.write("<!-- GENERATED by krs_quad_robot/scripts/normalize_urdf.py from "
                "krs_quad_robot.urdf. Do not edit by hand. -->\n")
        tree.write(f, encoding="unicode")
        f.write("\n")

    total = sum(r[1] for r in report)
    com = sum(r[1] * (frame[r[0]][1] + r[2]) for r in report) / total
    print("wrote {}".format(DST))
    print("total mass {:.1f} g, whole-robot CoM at q=0 (turned frame) "
          "x={:+.1f} y={:+.1f} z={:+.1f} mm".format(total * 1e3, *(com * 1e3)))
    for body, m, c in report:
        print("  {:10s} {:6.1f} g  com {}".format(body, m * 1e3, np.round(c * 1e3, 1)))
    print()
    print("joints (sign = normalised angle / CAD angle):")
    for body in bodies[1:]:
        info = joint_info[body]
        print("  {:14s} {:+d}  axis at {}".format(
            info["name"], info["sign"], np.round(info["pos"] * 1e3, 1)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
