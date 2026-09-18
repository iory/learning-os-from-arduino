#!/usr/bin/env python
"""What can this machine actually do? A scripted trot, no RL, straight on the MJCF.

Run this BEFORE tuning any reward. It answers two questions that reward shaping
cannot:

  1. Is walking physically possible with this asset, these actuators and this
     contact model? (If a hand-written gait cannot move the robot, no policy will.)
  2. What velocity range is reachable, and at what torque cost? That range is what
     the task's command distribution has to be set to -- asking a policy for a
     speed the hardware cannot reach means most of the command distribution earns
     no tracking reward no matter what the policy does, and "stand still" wins.

Method: a Cartesian foot trajectory per leg (stance = foot travels backwards at
the stance height, swing = forward with a sinusoidal lift), diagonal pairs in
antiphase, numerically inverse-kinematicked to (hip, knee) and written to the
MJCF's own <position> actuators at 50 Hz. So it goes through exactly the servo
model the RL task uses: stall-torque forcerange, back-EMF damping, armature.

Measured on this robot (6 s runs, statistics taken after the first second):

    stride  height   lift  period |  vx[m/s] | tau_max tau_p99 tau_rms | qd_p99 qd_rms
     0.080   0.140  0.030    0.50 |    0.138 |    2.94    2.94    1.64 |   4.98   2.48
     0.060   0.140  0.030    0.50 |    0.114 |    2.94    2.94    1.48 |   4.69   2.24
     0.060   0.140  0.020    0.70 |    0.086 |    2.43    2.01    0.89 |   2.84   1.32
     0.040   0.140  0.020    0.70 |    0.048 |    2.39    1.89    0.76 |   2.54   1.13

i.e. 0.05-0.14 m/s -- but the top of that band is pinned against the stall
torque, and even the mildest useful gait needs a 2.0 Nm 99th percentile. The
band that fits INSIDE the servo envelope is more like 0.05-0.09 m/s. 45 rpm
servos on a 0.19 m leg are simply slow.

Report the MAXIMUM over the run, not the value at the final step: an earlier
version of this script printed the last sample and made a 2.4 Nm gait look like
a 1.3 Nm one, which led to a torque threshold that suppressed walking entirely.

Usage:

    uv run --with mujoco --with numpy \
        python arduino_os_quad_robot/scripts/openloop_trot.py
"""
import numpy as np, mujoco, itertools, sys
X="${ARDUINO_QUAD_XML:-$(dirname "$0")/../mjcf/arduino_os_quad_robot.xml}"
LEGS=("FL","RL","RR","FR")
PHASE={"FL":0.0,"RL":0.5,"RR":0.0,"FR":0.5}      # diagonal trot
L1,L2=0.088,0.119                                 # sagittal thigh, shank
# toe offset in the shank direction at q=0 (measured): the shank is not exactly
# along the knee->toe line, so calibrate the IK against FK once.

m=mujoco.MjModel.from_xml_path(X)
d=mujoco.MjData(m)
jid={n:mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,n) for n in
     [f"{l}_{j}_joint" for l in LEGS for j in ("hip","knee")]}
qadr={n:m.jnt_qposadr[i] for n,i in jid.items()}
aid={}
for a in range(m.nu):
    nm=mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_ACTUATOR,a)
    aid[nm.replace("_act","")]=a
sid={l:mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_SITE,l) for l in LEGS}
bid=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_BODY,"base_link")

# FK/IK queries need their OWN MjData -- doing them on `d` would reset the
# running simulation on every control step (which is exactly the bug that made
# the first version of this script report vx = 0.000 for every parameter set).
dk = mujoco.MjData(m)

def toe_rel_hip(leg,hip,knee):
    """FK: toe position relative to the base, sagittal (x,z), for one leg."""
    mujoco.mj_resetData(m,dk)
    dk.qpos[:3]=0.0; dk.qpos[3]=1.0; dk.qpos[4:7]=0.0
    for l in LEGS:
        dk.qpos[qadr[f"{l}_hip_joint"]]=hip if l==leg else 0.0
        dk.qpos[qadr[f"{l}_knee_joint"]]=knee if l==leg else 0.0
    mujoco.mj_kinematics(m,dk)
    return dk.site_xpos[sid[leg]][[0,2]]-dk.xpos[bid][[0,2]]

# numeric IK on the (hip,knee) pair for a target toe position relative to base
def ik(leg,target,seed):
    q=np.array(seed,dtype=float)
    for _ in range(60):
        f=toe_rel_hip(leg,q[0],q[1])-target
        if np.linalg.norm(f)<1e-5: break
        J=np.zeros((2,2)); e=1e-5
        for i in range(2):
            qp=q.copy(); qp[i]+=e
            J[:,i]=(toe_rel_hip(leg,qp[0],qp[1])-toe_rel_hip(leg,q[0],q[1]))/e
        q-=np.linalg.solve(J+1e-9*np.eye(2),f)
        q=np.clip(q,-2.0,2.0)
    return q

def run(stride, height, lift, period, T=6.0, mu=None, verbose=False):
    if mu is not None:
        m.geom_friction[:,0]=mu
    mujoco.mj_resetDataKeyframe(m,d,0)
    d.qpos[2]=height+0.01
    # nominal toe under the hip at the requested height
    nom={l: np.array([toe_rel_hip(l,0.7748,-1.3474)[0], -height]) for l in LEGS}
    seed={l:(0.7748,-1.3474) for l in LEGS}
    n=int(T/m.opt.timestep); dec=4
    x0=None; xs=[]; QD=[]; TAU=[]; NET=[]; ZS=[]; TILTS=[]
    for k in range(n):
        if k%dec==0:
            t=k*m.opt.timestep
            for l in LEGS:
                ph=((t/period)+PHASE[l])%1.0
                if ph<0.5:                       # stance: foot moves backward
                    s=ph/0.5
                    px=nom[l][0]+stride*(0.5-s); pz=nom[l][1]
                else:                            # swing: forward + lift
                    s=(ph-0.5)/0.5
                    px=nom[l][0]+stride*(-0.5+s); pz=nom[l][1]+lift*np.sin(np.pi*s)
                q=ik(l,np.array([px,pz]),seed[l]); seed[l]=tuple(q)
                d.ctrl[aid[f"{l}_hip_joint"]]=q[0]
                d.ctrl[aid[f"{l}_knee_joint"]]=q[1]
        mujoco.mj_step(m,d)
        if k==int(1.0/m.opt.timestep): x0=d.qpos[0]
        xs.append(d.qpos[0])
        if k*m.opt.timestep>1.0:
            R2=np.zeros(9); mujoco.mju_quat2Mat(R2,d.qpos[3:7])
            TILTS.append(np.degrees(np.arccos(np.clip(R2.reshape(3,3)[2,2],-1,1))))
            ZS.append(d.qpos[2])
            QD.append(np.abs(d.qvel[6:]).copy())
            TAU.append(np.abs(d.actuator_force).copy())
            # NET torque delivered to the link = actuator output minus the passive
            # back-EMF damper we put on the joint. actuator_force alone counts the
            # damping compensation as load, which on this very light robot is most
            # of the number.
            NET.append(np.abs(d.actuator_force - m.dof_damping[6:]*d.qvel[6:]).copy())
    R=np.zeros(9); mujoco.mju_quat2Mat(R,d.qpos[3:7])
    tilt=np.degrees(np.arccos(np.clip(R.reshape(3,3)[2,2],-1,1)))
    v=(d.qpos[0]-x0)/(T-1.0)
    QD=np.array(QD); TAU=np.array(TAU); NET=np.array(NET)
    zs=np.array(ZS)
    return (v, d.qpos[2], tilt, zs.min(), zs.max(), np.array(TILTS).max(),
            TAU.max(), np.sqrt((TAU**2).mean()), np.percentile(TAU,99),
            QD.max(), np.sqrt((QD**2).mean()), np.percentile(QD,99),
            NET.max(), np.percentile(NET,99), np.sqrt((NET**2).mean()))

print("scripted diagonal trot on the MJCF (no RL)")
print("%7s %6s %7s | %8s | %6s %6s %6s | %6s %6s | %6s %6s %5s"
      % ("stride","lift","period","vx[m/s]",
         "z_min","z_max","tiltmx","NETp99","NETrms","qd_p99","qd%NL","OK"))
print("  (tau = actuator_force、NET = 逆起電力ダンパを引いた正味。"
      "連続定格 0.98 Nm と比べるべきなのは NET)")
best=None
import os as _os
_G = _os.environ.get("TROT_GRID", "default")
if _G == "back":      # 同じ組み立てで進行方向を逆にする (ストライド負)
    _grid = itertools.product((-0.03,-0.04,-0.05,-0.06,-0.07,-0.08),(0.02,),
                              (0.30,0.35,0.40,0.50,0.60),(0.14,))
elif _G == "safe":    # how fast can it go while keeping servo speed margin?
    _grid = itertools.product((0.03,0.04,0.05,0.06,0.07,0.08),(0.02,),
                              (0.30,0.35,0.40,0.50,0.60),(0.14,))
elif _G == "fast":    # push for speed: short periods, long strides
    _grid = itertools.product((0.06,0.08,0.10,0.12),(0.02,0.03),
                              (0.25,0.30,0.35,0.40),(0.14,))
else:
    _grid = itertools.product((0.04,0.06,0.08),(0.02,0.03),(0.5,0.7),(0.13,0.14))
for stride,lift,period,height in _grid:
    (v,z,tilt,zmin,zmax,tiltmx,tmx,trms,tp99,qmx,qrms,qp99,nmx,np99,nrms)=run(stride,height,lift,period)
    # 妥当性: 胴が跳ね上がったり潜ったりせず、姿勢が保たれていること。
    ok = (zmin > 0.5*height) and (zmax < 1.6*height) and (tiltmx < 25.0) and abs(v) > 0.01
    print("%7.3f %6.3f %7.2f | %8.3f | %6.3f %6.3f %5.1f° | %6.2f %6.2f | %6.2f %5.0f%% %5s"
          % (stride,lift,period,v,zmin,zmax,tiltmx,np99,nrms,qp99,100*qp99/4.717,
             "OK" if ok else "NG"))
    if not ok:
        continue
    if best is None or abs(v)>abs(best[0]): best=(v,stride,height,lift,period)
print("\nbest: vx=%.3f m/s at stride=%.3f height=%.3f lift=%.3f period=%.2f"%best)
