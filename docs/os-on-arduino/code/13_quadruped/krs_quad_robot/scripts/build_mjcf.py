#!/usr/bin/env python
"""Build the krs_quad_robot MJCF: normalised URDF -> mjcf_converter -> RL bits.

The KRS version counterpart of ``arduino_os_quad_robot/scripts/build_mjcf.py``.

  1. ``normalize_urdf.py``   CAD export -> nine links, FL/FR/RL/RR names, every
                             axis +Y, front = +X, measured masses (run first).
  2. ``skrobot.urdf.urdf_to_mjcf``  floating base, meshes -> STL, position
                             actuators with the URDF's effort as forcerange.
  3. this file's post-process:
       * the URDF's collision meshes are dropped (contacts use the primitives
         below; the visual meshes stay for rendering)
       * a contact sphere at each toe (``FL_toe`` ...) and a toe site (``FL`` ...)
       * ``base_box`` (the body's outline) and a ``<leg>_knee_ball`` at each
         knee, so belly and knee contacts with the floor exist and can be
         detected and punished
       * an IMU site on the base + gyro / velocimeter / accelerometer /
         subtree-angular-momentum sensors, named as the mjlab velocity task
         expects (``imu_ang_vel``, ``imu_lin_vel``, ``root_angmom``)
       * the ``home`` keyframe, solved here by leg IK (see HOME_BASE_HEIGHT)

Run from code/13_quadruped::

  uv run --no-project --with 'scikit-robot>=0.3.21' --with mujoco \
      python krs_quad_robot/scripts/build_mjcf.py
"""
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROBOT = os.path.dirname(HERE)

from skrobot.urdf import urdf_to_mjcf  # noqa: E402

URDF = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
    ROBOT, "urdf", "krs_quad_robot_rl.urdf")
MJCF = sys.argv[2] if len(sys.argv) > 2 else os.path.join(
    ROBOT, "mjcf", "krs_quad_robot.xml")

LEGS = ("FL", "RL", "RR", "FR")
BASE_BODY = "base_link"

# KRS-3302 ICS (https://kondo-robot.com/product/03146), 7.4 V: 0.657 Nm stall,
# 6.545 rad/s no load. Back-EMF modelled as joint damping = stall / no-load.
BACKEMF_DAMPING = 0.657 / 6.545
JOINT_ARMATURE = 7e-4   # ESTIMATE, see rl/krs_robot_cfg.py
ACTUATOR_KP = 5.5       # ESTIMATE, see rl/krs_robot_cfg.py

# Home stance: base_link origin (= the body's bottom face) this high, every toe
# directly under its own hip axis, knee TRAILING (behind the hip), the same
# topology as the Feetech robot. The leg reaches 0.208 m from hip axis to toe;
# at 0.13 m the hip-to-floor distance is 0.146 m, 70 % of full reach. Higher
# shortens the knee's moment arm (the KRS servo has little torque to spare);
# lower leaves more room to swing. Both joints stay far inside +-135 deg.
HOME_BASE_HEIGHT = float(os.environ.get("KRS_QUAD_HOME_HEIGHT", "0.13"))

TOE_RADIUS = 0.006  # the bracket's tip is 12 mm wide

# Body outline (body + lid, measured on the meshes in the CAD frame:
# x -89..87, y -55..55, z 0..78 mm). The lid is 3 mm narrower; one box is
# enough for "did the belly touch the floor".
BASE_BOX_HALF = (0.088, 0.055, 0.039)
BASE_BOX_POS = (0.0, 0.0, 0.039)
KNEE_BALL_RADIUS = 0.012  # the knee servo's case end around the axis

# Leg capsules, for leg-against-leg contact. Measured on the meshes in the
# body frames (thigh frame = hip axis, foot frame = knee axis, legs at q = 0):
#   thigh (leg link + knee servo): x +-13 mm, y 17..65 mm, z +12..-97 mm
#   shank (bracket):               y 3..16 mm, from the knee down to the toe
# The front and rear legs of one side swing in the same planes, so with the
# hips only 132 mm apart a front knee and a rear leg can meet. A capsule per
# segment, centred in the part's y band, with the part's half-width as radius.
THIGH_CAP_Y = 0.041
THIGH_CAP_RADIUS = 0.013
SHANK_CAP_Y = 0.0095
SHANK_CAP_RADIUS = 0.008


def toe_points(model, data, mujoco):
    """Centre of each foot's tip face at q = 0, in the foot body's frame.

    The tip is a 12 mm wide flat face, so the single lowest vertex lands on
    either edge depending on meshing noise (6 mm apart between legs); the
    centroid of every vertex within 0.5 mm of the lowest one is the face centre.
    """
    mujoco.mj_kinematics(model, data)
    out = {}
    for leg in LEGS:
        b = model.body("{}_foot".format(leg)).id
        best = None
        for g in range(model.ngeom):
            if model.geom_bodyid[g] != b or model.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH:
                continue
            m = model.geom_dataid[g]
            v = model.mesh_vert[model.mesh_vertadr[m]:model.mesh_vertadr[m] + model.mesh_vertnum[m]]
            vw = (data.geom_xmat[g].reshape(3, 3) @ v.T).T + data.geom_xpos[g]
            low = vw[vw[:, 2] < vw[:, 2].min() + 5e-4].mean(axis=0)
            if best is None or low[2] < best[2]:
                best = low
        R = data.xmat[b].reshape(3, 3)
        out[leg] = R.T @ (best - data.xpos[b])
    return out


def toe_centre(tip):
    """Contact sphere centre for a measured tip point (foot frame). The foot
    frame is parallel to the base at q = 0 (normalize_urdf.py), so lifting by
    the radius along local +Z makes the sphere tangent to the floor at the tip
    in the CAD pose."""
    return tip + np.array([0.0, 0.0, TOE_RADIUS])


def solve_home(model, mujoco, toe_local):
    """Hip/knee angles that put each toe sphere on the floor (centre one radius
    up) with its centre directly under the hip axis."""
    data = mujoco.MjData(model)
    home = {}
    for leg in LEGS:
        jh = model.joint("{}_hip_joint".format(leg))
        jk = model.joint("{}_knee_joint".format(leg))
        hip_body = model.body("{}_thigh".format(leg)).id
        foot = model.body("{}_foot".format(leg)).id
        q = np.array([0.6, -1.2])  # knee trailing: hip forward-positive, knee back

        def residual(q):
            data.qpos[:] = 0.0
            data.qpos[2] = HOME_BASE_HEIGHT
            data.qpos[3] = 1.0
            data.qpos[jh.qposadr[0]] = q[0]
            data.qpos[jk.qposadr[0]] = q[1]
            mujoco.mj_kinematics(model, data)
            toe = data.xpos[foot] + data.xmat[foot].reshape(3, 3) @ toe_centre(toe_local[leg])
            return np.array([toe[0] - data.xpos[hip_body][0], toe[2] - TOE_RADIUS])

        for _ in range(50):
            r = residual(q)
            if np.abs(r).max() < 1e-7:
                break
            J = np.zeros((2, 2))
            for k in range(2):
                dq = np.zeros(2)
                dq[k] = 1e-6
                J[:, k] = (residual(q + dq) - r) / 1e-6
            q = q - np.linalg.solve(J, r)
        assert np.abs(residual(q)).max() < 1e-6, (leg, q, residual(q))
        home["{}_hip_joint".format(leg)] = float(q[0])
        home["{}_knee_joint".format(leg)] = float(q[1])
    return home


def post_process(mjcf_path, toe_local, home):
    tree = ET.parse(mjcf_path)
    root = tree.getroot()
    bodies = {b.get("name"): b for b in root.iter("body")}

    # Collision meshes out: the task collides with primitives only.
    for body in bodies.values():
        for g in list(body.findall("geom")):
            if "collision" in (g.get("name") or ""):
                body.remove(g)

    def geom(parent, **attrs):
        ET.SubElement(parent, "geom", {k: str(v) for k, v in attrs.items()})

    for leg in LEGS:
        foot = bodies["{}_foot".format(leg)]
        # The foot frame is parallel to the base at q = 0 (normalize_urdf.py),
        # so world +Z is local +Z there: lift the centre by the radius so the
        # sphere is tangent to the floor at the measured tip.
        centre = toe_centre(toe_local[leg])
        geom(foot, name="{}_toe".format(leg), type="sphere", size="{:g}".format(TOE_RADIUS),
             pos="{:.6g} {:.6g} {:.6g}".format(*centre), rgba="0.1 0.1 0.1 1")
        ET.SubElement(foot, "site", {
            "name": leg, "pos": "{:.6g} {:.6g} {:.6g}".format(*toe_local[leg]),
            "size": "0.004", "group": "4", "rgba": "1 0 0 1"})
        # knee ball on the knee axis. The foot frame origin is at the horn, on
        # the outer face; the servo case sits 12 mm further in.
        y = -0.012 if leg.endswith("L") else 0.012
        geom(foot, name="{}_knee_ball".format(leg), type="sphere",
             size="{:g}".format(KNEE_BALL_RADIUS), pos="0 {:g} 0".format(y),
             group="3", rgba="0.8 0.2 0.2 0.3")
        side = 1.0 if leg.endswith("L") else -1.0
        thigh = bodies["{}_thigh".format(leg)]
        knee = [float(v) for v in foot.get("pos").split()]
        geom(thigh, name="{}_thigh_cap".format(leg), type="capsule",
             size="{:g}".format(THIGH_CAP_RADIUS),
             fromto="0 {y:.6g} 0 {x:.6g} {y:.6g} {z:.6g}".format(
                 x=knee[0], y=side * THIGH_CAP_Y, z=knee[2]),
             group="3", rgba="0.2 0.8 0.2 0.3")
        ys = side * SHANK_CAP_Y
        geom(foot, name="{}_shank_cap".format(leg), type="capsule",
             size="{:g}".format(SHANK_CAP_RADIUS),
             fromto="0 {:.6g} 0 {:.6g} {:.6g} {:.6g}".format(
                 ys, centre[0], ys, centre[2]),
             group="3", rgba="0.2 0.8 0.2 0.3")

    base = bodies[BASE_BODY]
    geom(base, name="base_box", type="box",
         size="{:g} {:g} {:g}".format(*BASE_BOX_HALF),
         pos="{:g} {:g} {:g}".format(*BASE_BOX_POS), group="3", rgba="0.2 0.2 0.8 0.3")
    ET.SubElement(base, "site", {"name": "imu_in_body", "pos": "0 0 0",
                                 "size": "0.005", "group": "4", "rgba": "0 1 0 1"})

    # mjlab's velocity task reads these by name.
    if root.find("sensor") is None:
        sensor = ET.SubElement(root, "sensor")
        ET.SubElement(sensor, "gyro", {"name": "imu_ang_vel", "site": "imu_in_body"})
        ET.SubElement(sensor, "velocimeter", {"name": "imu_lin_vel", "site": "imu_in_body"})
        ET.SubElement(sensor, "accelerometer", {"name": "imu_lin_acc", "site": "imu_in_body"})
        ET.SubElement(sensor, "subtreeangmom", {"name": "root_angmom", "body": BASE_BODY})

    # The converter wrote a keyframe from the placeholder home; replace it.
    for kf in root.findall("keyframe"):
        root.remove(kf)
    joint_order = [j.get("name") for j in root.find("worldbody").iter("joint")
                   if j.get("type") == "hinge"]
    qpos = [0.0, 0.0, HOME_BASE_HEIGHT, 1.0, 0.0, 0.0, 0.0] + [home[j] for j in joint_order]
    kf = ET.SubElement(root, "keyframe")
    act = root.find("actuator")
    ctrl = [home[a.get("joint")] for a in act] if act is not None else []
    ET.SubElement(kf, "key", {"name": "home",
                              "qpos": " ".join("{:.8g}".format(v) for v in qpos),
                              "ctrl": " ".join("{:.8g}".format(v) for v in ctrl)})

    ET.indent(tree, space="  ")
    tree.write(mjcf_path, encoding="unicode", xml_declaration=False)


def main():
    import mujoco

    if not os.path.exists(URDF):
        raise SystemExit("{} is missing -- run normalize_urdf.py first".format(URDF))
    os.makedirs(os.path.dirname(MJCF), exist_ok=True)
    zero = {"{}_{}_joint".format(leg, j): 0.0 for leg in LEGS for j in ("hip", "knee")}
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
        home=zero,
        home_base_height=HOME_BASE_HEIGHT,
    )
    print("stage 1: converted URDF -> {}".format(MJCF))

    model = mujoco.MjModel.from_xml_path(MJCF)
    toe_local = toe_points(model, mujoco.MjData(model), mujoco)
    home = solve_home(model, mujoco, toe_local)
    post_process(MJCF, toe_local, home)
    print("stage 2: toes (r={:g} m), base box, knee balls, IMU site/sensors, home key"
          .format(TOE_RADIUS))

    # --- report ---------------------------------------------------------------
    model = mujoco.MjModel.from_xml_path(MJCF)
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)
    mujoco.mj_forward(model, data)
    print("mass {:.1f} g, nbody {}, ngeom {}".format(
        model.body_subtreemass[0] * 1e3, model.nbody, model.ngeom))
    print("home (base {:.3f} m):".format(HOME_BASE_HEIGHT))
    for leg in LEGS:
        h, k = home["{}_hip_joint".format(leg)], home["{}_knee_joint".format(leg)]
        toe = data.geom_xpos[model.geom("{}_toe".format(leg)).id]
        knee = data.xpos[model.body("{}_foot".format(leg)).id]
        print("  {}  hip {:+.4f} ({:+.1f} deg)  knee {:+.4f} ({:+.1f} deg)  "
              "knee at x {:+.1f} z {:.1f} mm, toe z {:.1f} mm".format(
                  leg, h, np.degrees(h), k, np.degrees(k),
                  knee[0] * 1e3, knee[2] * 1e3, (toe[2] - TOE_RADIUS) * 1e3))
        assert abs(h) < 0.75 * 2.356194 and abs(k) < 0.75 * 2.356194
    return 0


if __name__ == "__main__":
    sys.exit(main())
