"""Xandra-Lite: reduced physical body for the Game prototype (MuJoCo primitives only, no clips).

World: Z up, she faces +X, her left is +Y. Units: m, kg, s, N, N*m.

Bodies (60 kg total; dimensions from the Xandra physics model):
  torso  free joint; pelvis + trunk + head as ONE rigid body (no spine to collapse)  32.6 kg
  arms   both arms + hands as one link on a shoulder pitch hinge                      6.5 kg
  thigh_l/r  hip roll + hip pitch                                                    6.5 kg each
  shank_l/r  slide joint along the leg = leg length (crouch, push, jump)             3.0 kg each
  foot_l/r   ankle pitch + ankle roll; flat box sole 0.24 x 0.08 m, heel 6 cm behind the ankle    0.95 kg each

Actuators (11 motors, explicit force/torque limits = ASSUMPTIONS, logged):
  hip_roll_l/r, hip_pitch_l/r   +-120 N*m   (as the 33-motor model's hips)
  leg_l/r                       +-900 N     (~1.5 body weight per leg)
  ankle_l/r (pitch)             +-80 N*m    (the 33-motor model had 40; 80 is closer to an adult)
  ankle_roll_l/r                +-60 N*m    (needed: without it two flat feet and an upright body
                                             cannot shift the COM sideways at all - measured)
  shoulder                      +-60 N*m

Obstacles are plain world geoms/bodies passed in by the caller; nothing switches collisions.
"""
from __future__ import annotations

import mujoco

HIP_Z = 0.90          # hip joint height when standing at full leg length (= LEG_LEN + ANKLE_Z)
LEG_LEN = 0.82        # hip -> ankle at full length
ANKLE_Z = 0.08        # ankle above the sole
SOLE = (0.24, 0.08, 0.03)  # length, width, thickness
HEEL = 0.06           # sole extends this far behind the ankle
SHOULDER_Z = 0.43     # above the hip centre
ARM_LEN = 0.56
LEG_MAX_SHORTEN = 0.40

# Geometry variants (compared in isolation; only these three numbers differ):
#   v0  original: feet 18 cm apart (hips +-0.09), sole width 8 cm, thigh radius 0.065
#   v1  step width 12 cm (hips +-0.06), sole width 10 cm, thigh radius 0.055 (else the thighs overlap)
GEOMETRY = {"v0": dict(hip_y=0.09, sole_w=0.08, thigh_r=0.065),
            "v1": dict(hip_y=0.06, sole_w=0.10, thigh_r=0.055)}

ACTUATORS = ["hip_roll_l", "hip_pitch_l", "leg_l", "ankle_l", "ankle_roll_l",
             "hip_roll_r", "hip_pitch_r", "leg_r", "ankle_r", "ankle_roll_r", "shoulder"]
LIMITS = {"hip_roll": 120.0, "hip_pitch": 120.0, "leg": 900.0, "ankle": 80.0, "ankle_roll": 60.0, "shoulder": 60.0}


# Position servos (path 2): ctrl = joint target; force = kp*(target-q) - kv*qdot, clipped to the same
# force/torque limits as the motors. kv is integrated implicitly by MuJoCo (implicitfast), so stiff
# servos stay numerically stable on light links. Gains are ASSUMPTIONS (servo stiffness).
SERVO_GAINS = {"hip_roll": (1500.0, 60.0), "hip_pitch": (1500.0, 60.0), "leg": (40000.0, 1500.0),
               "ankle": (1200.0, 40.0), "ankle_roll": (1000.0, 30.0), "shoulder": (300.0, 15.0)}


def limit_key(actuator: str) -> str:
    return actuator if actuator == "shoulder" else actuator.rsplit("_", 1)[0]


def _leg(side: str, y: float, sole_w: float, thigh_r: float) -> str:
    sl, _sw, st = SOLE
    sw = sole_w
    fx = sl / 2 - HEEL  # sole centre relative to the ankle
    thigh_len = LEG_LEN / 2
    return f"""
      <body name="thigh_{side}" pos="0 {y} 0">
        <joint name="hip_roll_{side}" type="hinge" axis="1 0 0" range="-0.5 0.5" damping="1"/>
        <joint name="hip_pitch_{side}" type="hinge" axis="0 -1 0" range="-0.9 1.9" damping="1"/>
        <geom name="thigh_{side}" type="capsule" fromto="0 0 0 0 0 {-thigh_len:.3f}" size="{thigh_r}" mass="6.5"/>
        <body name="shank_{side}" pos="0 0 {-thigh_len:.3f}">
          <joint name="leg_{side}" type="slide" axis="0 0 1" range="0 {LEG_MAX_SHORTEN}" damping="20"/>
          <geom name="shank_{side}" type="capsule" fromto="0 0 0 0 0 {-(LEG_LEN - thigh_len):.3f}" size="0.045" mass="3.0"/>
          <body name="foot_{side}" pos="0 0 {-(LEG_LEN - thigh_len):.3f}">
            <joint name="ankle_{side}" type="hinge" axis="0 -1 0" range="-0.8 0.8" damping="0.5"/>
            <joint name="ankle_roll_{side}" type="hinge" axis="1 0 0" range="-0.35 0.35" damping="0.5"/>
            <geom name="sole_{side}" type="box" pos="{fx:.3f} 0 {-(ANKLE_Z - st / 2):.3f}" size="{sl / 2} {sw / 2} {st / 2}" mass="0.95" friction="1.0 0.02 0.01"/>
            <site name="sole_{side}" pos="{fx:.3f} 0 {-ANKLE_Z:.3f}"/>
          </body>
        </body>
      </body>"""


JOINT_RANGES = {"hip_roll": (-0.5, 0.5), "hip_pitch": (-0.9, 1.9), "leg": (0.0, LEG_MAX_SHORTEN),
                "ankle": (-0.8, 0.8), "ankle_roll": (-0.35, 0.35), "shoulder": (-0.8, 3.2)}


def build_xml(world: str = "", extra_actuators: str = "", start=(0.0, 0.0, 0.0), servo: bool = True,
              geometry: str = "v0") -> str:
    """MJCF for Xandra-Lite standing at start=(x, y, ground z) plus caller-supplied world geometry."""
    g = GEOMETRY[geometry]
    start = tuple(start) + (0.0,) * (3 - len(start))
    stand_z = HIP_Z + start[2]
    if servo:
        motors = "\n".join(
            f'    <position name="{a}" joint="{a}" kp="{SERVO_GAINS[limit_key(a)][0]}" kv="{SERVO_GAINS[limit_key(a)][1]}" '
            f'ctrlrange="{JOINT_RANGES[limit_key(a)][0]} {JOINT_RANGES[limit_key(a)][1]}" '
            f'forcelimited="true" forcerange="{-LIMITS[limit_key(a)]} {LIMITS[limit_key(a)]}"/>' for a in ACTUATORS)
    else:
        motors = "\n".join(
            f'    <motor name="{a}" joint="{a}" ctrlrange="{-LIMITS[limit_key(a)]} {LIMITS[limit_key(a)]}"/>' for a in ACTUATORS)
    return f"""<mujoco model="xandra_lite">
  <compiler angle="radian" autolimits="true"/>
  <option timestep="0.002" gravity="0 0 -9.81" integrator="implicitfast" cone="elliptic" impratio="10"/>
  <visual><global offwidth="1280" offheight="720"/></visual>
  <default>
    <geom condim="3" friction="1.0 0.02 0.01" solref="0.004 1" solimp="0.95 0.99 0.001" rgba="0.75 0.6 0.5 1"/>
    <motor ctrllimited="true"/>
  </default>
  <asset>
    <texture name="grid" type="2d" builtin="checker" rgb1=".85 .85 .82" rgb2=".75 .75 .72" width="512" height="512"/>
    <material name="grid" texture="grid" texrepeat="20 20"/>
  </asset>
  <worldbody>
    <light pos="0 -3 5" dir="0 0.5 -1" diffuse="0.8 0.8 0.8"/>
    <geom name="floor" type="plane" size="30 30 0.1" material="grid"/>
    {world}
    <body name="torso" pos="{start[0]} {start[1]} {stand_z}">
      <freejoint name="root"/>
      <geom name="pelvis" type="capsule" fromto="0 -0.09 0.02 0 0.09 0.02" size="0.09" mass="9.3" rgba="0.5 0.45 0.7 1"/>
      <geom name="trunk" type="capsule" fromto="0 0 0.10 0 0 0.40" size="0.12" mass="18.0" rgba="0.5 0.6 0.55 1"/>
      <geom name="head" type="sphere" pos="0 0 0.60" size="0.09" mass="5.3" rgba="0.9 0.75 0.65 1"/>
      <site name="torso" pos="0 0 0"/>
      <body name="arms" pos="0 0 {SHOULDER_Z}">
        <joint name="shoulder" type="hinge" axis="0 -1 0" range="-0.8 3.2" damping="0.5"/>
        <geom name="arm_l" type="capsule" fromto="0 0.22 0 0 0.22 {-ARM_LEN + 0.04:.3f}" size="0.035" mass="3.0" rgba="0.55 0.45 0.75 1"/>
        <geom name="arm_r" type="capsule" fromto="0 -0.22 0 0 -0.22 {-ARM_LEN + 0.04:.3f}" size="0.035" mass="3.0" rgba="0.55 0.45 0.75 1"/>
        <geom name="hand_l" type="sphere" pos="0 0.22 {-ARM_LEN:.3f}" size="0.04" mass="0.25" rgba="0.9 0.75 0.65 1"/>
        <geom name="hand_r" type="sphere" pos="0 -0.22 {-ARM_LEN:.3f}" size="0.04" mass="0.25" rgba="0.9 0.75 0.65 1"/>
        <site name="hand_l" pos="0 0.22 {-ARM_LEN:.3f}"/>
        <site name="hand_r" pos="0 -0.22 {-ARM_LEN:.3f}"/>
      </body>
      {_leg("l", g["hip_y"], g["sole_w"], g["thigh_r"])}
      {_leg("r", -g["hip_y"], g["sole_w"], g["thigh_r"])}
    </body>
  </worldbody>
  <contact>
    <exclude body1="torso" body2="thigh_l"/>
    <exclude body1="torso" body2="thigh_r"/>
    <exclude body1="thigh_l" body2="foot_l"/>
    <exclude body1="thigh_r" body2="foot_r"/>
    <exclude body1="torso" body2="arms"/>
    <!-- the arm link has no shoulder abduction, so a thigh swinging sideways would hit a rigid wall;
         a real arm is simply pushed aside. Arm-thigh contacts are excluded (measured: they blocked
         the hip roll at its 120 N*m limit during the weight shift). -->
    <exclude body1="arms" body2="thigh_l"/>
    <exclude body1="arms" body2="thigh_r"/>
  </contact>
  <actuator>
{motors}
{extra_actuators}
  </actuator>
</mujoco>"""


def build(world: str = "", extra_actuators: str = "", start=(0.0, 0.0, 0.0), servo: bool = True,
          geometry: str = "v0") -> mujoco.MjModel:
    return mujoco.MjModel.from_xml_string(build_xml(world, extra_actuators, start, servo, geometry))
