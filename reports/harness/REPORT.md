# HARNESS validation

Physical support only. The harness writes `data.xfrc_applied` on the pelvis
before `mj_step`. It does not write `qpos` or `qvel`, does not weld the root,
does not disable gravity, and does not change actuator limits or the 33-channel command.

## Where the wrench goes

- Body name: `pelvis`
- Body id: **1**
- Point: that body's center of mass (MuJoCo's definition of `xfrc_applied`)
- Frame: **world**
- Layout on MuJoCo 3.14, checked against `mj_applyFT`: **force then torque**
  `[fx, fy, fz, tx, ty, tz]`. (Some notes say torque-then-force; this build does not.)

## Formula

Pelvis origin pose is the root free joint (`body_quat` is identity).
`qvel` is world `[vx, vy, vz, wx, wy, wz]`. `up` is the pelvis +Z axis in world.
`z_des` is the stand / `qpos0` pelvis height, captured once at enable.

```
Fz = clip(kp_z * (z_des - z) - kd_z * vz, -F_max, F_max)
Fxy = -kd_xy * vxy          # no position spring
Fxy *= Fxy_max / |Fxy|      if |Fxy| > Fxy_max
τ_rp = kp_rp * (up × ez)    # ez = (0,0,1); yaw component of this term is 0
τ   = τ_rp - (kd_rp * wx, kd_rp * wy, kd_yaw * wz)
τ  *= T_max / |τ|           if |τ| > T_max
```

Constants used on this model:

- `kp_z` = 30000.0 N/m
- `kd_z` = 1600.0 N/(m/s)
- `kp_rp` = 1800.0 N·m per unit sin(tilt)
- `kd_rp` = 90.0 N·m/(rad/s)
- `kd_yaw` = 6.0 N·m/(rad/s) (yaw spring is 0)
- `kd_xy` = 25.0 N/(m/s)
- `|Fz|` cap = 2200.0 N
- `|Fxy|` cap = 400.0 N
- `|τ|` cap = 350.0 N·m

Actuators: the same 33 motors. `u_cmd` is copied to `data.ctrl` by the existing
RAW actuator path. The harness does not add joint torques and does not edit `ctrlrange`.
Arms are not controlled by the harness. It is not a gait.

## Results

All checks passed: **True**

| # | Check | Result |
| --- | --- | --- |
| 1_hold_vs_raw | PASS |
| 2_lift_one_leg | PASS |
| 3_return_foot_contact | PASS |
| 4_xy_not_pinned | PASS |
| 5_gravity_on | PASS |
| 6_real_contacts | PASS |
| 7_disable_falls | PASS |
| 8_raw_path_no_harness | PASS |
| 9_assisted_still_present | PASS |
| apply_does_not_write_qpos_qvel | PASS |
| actuator_ctrlrange_unchanged | PASS |

### 1. Supported vs raw

- Harness on, zero actuator commands, 5.0 s: min pelvis z = **0.8942 m**,
  end z = **0.9021 m**, max tilt = **0.1168 rad**,
  end tilt = **0.0535 rad**.
- z samples (every 0.5 s): [0.9269, 0.9027, 0.9015, 0.9042, 0.9026, 0.8988, 0.8995, 0.9023, 0.9053, 0.9049]
- RAW, same model, harness off, xfrc held at 0, gravity on: pelvis z fell below
  0.55 m at **0.482 s** (z then 0.5487 m).
- End of harness hold still had **9** MuJoCo contacts, including floor pairs [['floor', 'foot_l_geom'], ['floor', 'foot_l_geom'], ['floor', 'foot_l_geom'], ['floor', 'foot_r_geom']].

### 2–3. One leg lifts and returns

- Open-loop `70.0` N·m on `knee_l_motor` only (other actuators 0).
- At t = 0.800 s the left sole was **0.508 m** off the floor,
  left foot–floor contacts = **0**,
  right (stance) contacts = **1**, stance normal force = **50.80 N**.
- Pelvis z during that sample: 0.898 m. Right sole z: 0.0060 m.
- After the knee command was set back to 0, left foot–floor contact returned at
  t = **1.110 s**, sole min z = **0.0009 m**,
  left contacts = **1** (normal **5996.36 N**),
  total `data.ncon` = **4**.
- Contact geoms: [['floor', 'foot_l_geom']].

### 4. XY is not pinned

- Validator-only probe: +80.0 N world X added **after** the harness write, for 1.5 s.
- Pelvis translation: **Δx = 0.2008 m**, **Δy = -0.1883 m** (not clamped to 0).
- With zero commands and no probe, pelvis still drifted by Δxy = (0.0523, -0.2042) m
  over 5.0 s. Horizontal support is damping, not a position spring.

### 5–7. Gravity, contacts, disable

- `model.opt.gravity` = **[0.0, 0.0, -9.81]** (unchanged from the XML default).
- Contacts above are `data.ncon` / `data.contact` geom ids, with `mj_contactForce` normal components. Not flags.
- Harness disabled at pelvis z = **0.9015 m** (xfrc set to 0, no qpos rewrite).
  After 1.2 s of ordinary physics: z = **0.4015 m** (drop **0.5000 m**).
  A twin run left on for 2.2 s ended at z = **0.8993 m**.

### 8–9. RAW path and Debug Assisted

- `Controller` in dynamic mode, harness never enabled, 400 ticks: max |xfrc_applied| = **0.0**,
  `harness.enabled` = **False**.
- `./START.command experiment` / `start-actuator-runtime` do not pass `--harness`.
  `tick()` calls `harness.apply` only inside `if self.harness.enabled`.
- `assisted_stand.AssistedStandController` still imports. `viewer/console.html` still has
  Position, Torque, and Assisted. Debug mode values `kinematic` / `dynamic` / `assisted` are unchanged.

## How to launch

```sh
./START.command harness
# menu item 3, or the same command
# clean observation viewer: http://127.0.0.1:8788/
# motors: ./humanoid act <33 floats>    ./humanoid zero
```

Debug (`./START.command debug`) is the developer console and still has Assisted.
Experiment (`./START.command experiment`) is RAW: no harness wrench.
`./START.command assisted` still starts the old assisted-stand runtime, but it is not in the menu.

## Limits

- The harness holds the pelvis up and keeps roll/pitch from a catastrophic fall.
  It does not balance the robot, time steps, or move the arms.
- Feet can slip. XY is supposed to be free, so the body can wander on the floor.
- A hard foot landing can spike contact force. That spike is MuJoCo contact, not a fake flag.
- Yaw is only lightly damped. There is no heading hold.
- Gains are for this mesh and this mass (~89 kg). They are constants in `physics/harness.py`.

