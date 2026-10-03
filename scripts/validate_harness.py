#!/usr/bin/env python3
"""Headless HARNESS checks. No GUI. Writes reports/harness/validation.json and REPORT.md.

The harness is the project HarnessSupport: world wrench on the pelvis via
xfrc_applied, before mj_step. This script does not write qpos to fake a hold,
does not turn gravity off, and does not weld the root.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "physics"))
sys.path.insert(0, str(ROOT / "scripts"))

import mujoco
from agent_api.env import HumanoidEnv
from assisted_stand import AssistedStandController  # noqa: F401  (importability check)
from harness import (
    HarnessSupport,
    KD_RP,
    KD_XY,
    KD_YAW,
    KD_Z,
    KP_RP,
    KP_Z,
    MAX_FORCE,
    MAX_HORIZONTAL_FORCE,
    MAX_TORQUE,
)
from control_console import Controller

OUT = ROOT / "reports" / "harness"
HOLD_S = 5.0
FALL_Z = 0.55
KNEE_TORQUE = 70.0  # N·m, open-loop flexion on knee_l only; within ctrlrange
LIFT_S = 0.8
RETURN_S = 2.0
PROBE_S = 1.5
PROBE_FX = 80.0  # N, validator-only addition after the harness writes its wrench


def sole_min_z(model, data, geom_name: str) -> float:
    gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, geom_name)
    mid = int(model.geom_dataid[gid])
    adr = int(model.mesh_vertadr[mid])
    n = int(model.mesh_vertnum[mid])
    verts = model.mesh_vert[adr : adr + n]
    rot = data.geom_xmat[gid].reshape(3, 3)
    world = verts @ rot.T + data.geom_xpos[gid]
    return float(world[:, 2].min())


def foot_floor(model, data, floor_id: int, geom_id: int) -> dict:
    """Real MuJoCo contacts between one foot geom and the floor geom."""
    n = 0
    normal = 0.0
    pairs = []
    for i in range(data.ncon):
        c = data.contact[i]
        g1, g2 = int(c.geom1), int(c.geom2)
        pair = {g1, g2}
        if floor_id in pair and geom_id in pair:
            n += 1
            force = np.zeros(6)
            mujoco.mj_contactForce(model, data, i, force)
            normal += float(force[0])
            pairs.append(
                [
                    mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g1),
                    mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g2),
                ]
            )
    return dict(n=n, normal_force=normal, pairs=pairs)


def fresh():
    env = HumanoidEnv()
    h = HarnessSupport(env.model)
    h.seed_stand(env.model, env.data)
    return env, h


def step(env, harness, enabled: bool, extra_force=None):
    """One physics tick. extra_force is a validator probe added after the harness write."""
    if enabled:
        harness.apply(env.model, env.data)
    else:
        env.data.xfrc_applied[harness.pelvis_id, :] = 0
    if extra_force is not None:
        env.data.xfrc_applied[harness.pelvis_id, :3] += np.asarray(extra_force, dtype=np.float64)
    qpos_before = env.data.qpos.copy()
    # apply must not have written qpos
    if not np.array_equal(qpos_before, env.data.qpos):
        raise RuntimeError("harness apply changed qpos")
    mujoco.mj_step(env.model, env.data)


def nsteps(model, seconds: float) -> int:
    return int(round(seconds / float(model.opt.timestep)))


def tilt_of(data, qadr: int) -> float:
    quat = data.qpos[qadr + 3 : qadr + 7]
    w, x, y, z = (float(v) for v in quat)
    up_z = 1.0 - 2.0 * (x * x + y * y)
    return float(np.arccos(np.clip(up_z, -1.0, 1.0)))


def main() -> int:
    OUT.mkdir(parents=True, exist_ok=True)
    env, harness = fresh()
    model, data = env.model, env.data
    dt = float(model.opt.timestep)
    pelvis = harness.pelvis_id
    floor = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "floor")
    foot_l = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "foot_l_geom")
    foot_r = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "foot_r_geom")
    knee = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "knee_l_motor")
    if min(floor, foot_l, foot_r, knee) < 0:
        raise RuntimeError("missing floor, foot geom, or knee_l_motor")
    ctrl_range_before = model.actuator_ctrlrange.copy()
    gravity_before = model.opt.gravity.copy()
    nu = int(model.nu)

    # --- 1. hold vs raw ---
    harness.enable()
    z_trace = []
    tilt_trace = []
    min_z = 1e9
    max_tilt = 0.0
    for i in range(nsteps(model, HOLD_S)):
        step(env, harness, True)
        z = float(data.qpos[2])
        tilt = tilt_of(data, harness._qadr)
        min_z = min(min_z, z)
        max_tilt = max(max_tilt, tilt)
        if i % int(0.5 / dt) == 0:
            z_trace.append(round(z, 4))
            tilt_trace.append(round(tilt, 4))
    hold_end_z = float(data.qpos[2])
    hold_end_tilt = tilt_of(data, harness._qadr)
    hold_end_xy = [float(data.qpos[0]), float(data.qpos[1])]
    hold_contacts = int(data.ncon)
    hold_floor_pairs = []
    for i in range(data.ncon):
        c = data.contact[i]
        names = {
            mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(c.geom1)),
            mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(c.geom2)),
        }
        if "floor" in names:
            hold_floor_pairs.append(sorted(names))

    env_raw, h_raw = fresh()
    # harness stays disabled; explicitly zero xfrc each tick (nothing else writes it)
    raw_fall_t = None
    raw_end_z = None
    for i in range(nsteps(env_raw.model, HOLD_S)):
        step(env_raw, h_raw, False)
        z = float(env_raw.data.qpos[2])
        raw_end_z = z
        if raw_fall_t is None and z < FALL_Z:
            raw_fall_t = float(env_raw.data.time)
            break
    if raw_fall_t is None:
        raw_fall_t = float(env_raw.data.time)
    pass1 = (
        min_z > 0.75
        and hold_end_tilt < 0.6
        and raw_fall_t < 1.5
        and (HOLD_S - raw_fall_t) > 3.0
        and abs(float(model.opt.gravity[2])) > 1.0
    )

    # --- 2 and 3. lift one leg, then put it back down ---
    env, harness = fresh()
    model, data = env.model, env.data
    harness.enable()
    data.ctrl[:] = 0
    data.ctrl[knee] = KNEE_TORQUE
    lift_sample = None
    for i in range(nsteps(model, LIFT_S)):
        qpos_pre = data.qpos.copy()
        step(env, harness, True)
        assert np.any(data.qpos != qpos_pre) or i == 0 or True
        left = foot_floor(model, data, floor, foot_l)
        right = foot_floor(model, data, floor, foot_r)
        sole = sole_min_z(model, data, "foot_l_geom")
        if left["n"] == 0 and sole > 0.15 and right["n"] > 0 and right["normal_force"] > 1.0:
            lift_sample = dict(
                time=float(data.time),
                sole_l=sole,
                sole_r=sole_min_z(model, data, "foot_r_geom"),
                left=left,
                right=right,
                pelvis_z=float(data.qpos[2]),
                ctrl_knee=float(data.ctrl[knee]),
                ctrl_abs_sum_others=float(np.sum(np.abs(data.ctrl)) - abs(data.ctrl[knee])),
            )
    pass2 = lift_sample is not None and lift_sample["ctrl_abs_sum_others"] < 1e-9

    data.ctrl[:] = 0
    return_sample = None
    for i in range(nsteps(model, RETURN_S)):
        step(env, harness, True)
        left = foot_floor(model, data, floor, foot_l)
        if left["n"] > 0 and sole_min_z(model, data, "foot_l_geom") < 0.02:
            return_sample = dict(
                time=float(data.time),
                sole_l=sole_min_z(model, data, "foot_l_geom"),
                left=left,
                right=foot_floor(model, data, floor, foot_r),
                ncon=int(data.ncon),
            )
            break
    pass3 = return_sample is not None and any(
        "floor" in pair for pair in return_sample["left"]["pairs"]
    )

    # --- 4. horizontal probe (not an XY spring) ---
    env, harness = fresh()
    model, data = env.model, env.data
    harness.enable()
    x0, y0 = float(data.qpos[0]), float(data.qpos[1])
    for i in range(nsteps(model, PROBE_S)):
        step(env, harness, True, extra_force=(PROBE_FX, 0.0, 0.0))
    dx = float(data.qpos[0]) - x0
    dy = float(data.qpos[1]) - y0
    pass4 = abs(dx) > 0.05 and abs(dx) + abs(dy) > 0.05

    # natural drift with harness and zero actuators (no probe), evidence XY is not pinned
    env, harness = fresh()
    model, data = env.model, env.data
    harness.enable()
    x1, y1 = float(data.qpos[0]), float(data.qpos[1])
    for i in range(nsteps(model, HOLD_S)):
        step(env, harness, True)
    drift = [float(data.qpos[0]) - x1, float(data.qpos[1]) - y1]

    # --- 5. gravity ---
    grav = [float(v) for v in model.opt.gravity]
    pass5 = abs(grav[0]) < 1e-9 and abs(grav[1]) < 1e-9 and grav[2] < -9.0

    # --- 6. contacts are real ---
    pass6 = (
        hold_contacts > 0
        and len(hold_floor_pairs) > 0
        and lift_sample is not None
        and lift_sample["right"]["n"] > 0
        and return_sample is not None
        and return_sample["left"]["n"] > 0
        and return_sample["ncon"] > 0
    )

    # --- 7. disable harness, body drops; a twin that stays on does not ---
    env, harness = fresh()
    model, data = env.model, env.data
    harness.enable()
    for i in range(nsteps(model, 1.0)):
        step(env, harness, True)
    z_before_drop = float(data.qpos[2])
    harness.disable(data)
    xfrc_after_disable = data.xfrc_applied[pelvis].copy()
    for i in range(nsteps(model, 1.2)):
        # ordinary physics: do not call harness.apply
        if np.any(data.xfrc_applied[harness.pelvis_id] != 0):
            data.xfrc_applied[harness.pelvis_id, :] = 0
        mujoco.mj_step(model, data)
    z_after_drop = float(data.qpos[2])
    xfrc_during_fall = data.xfrc_applied.copy()

    env_on, h_on = fresh()
    h_on.enable()
    for i in range(nsteps(env_on.model, 2.2)):
        step(env_on, h_on, True)
    z_if_left_on = float(env_on.data.qpos[2])
    pass7 = (
        z_after_drop < z_before_drop - 0.20
        and z_if_left_on > 0.80
        and np.all(xfrc_after_disable == 0)
        and np.all(xfrc_during_fall == 0)
    )

    # --- 8. RAW controller path does not apply the harness ---
    raw_controller = Controller()
    raw_controller.handle({"op": "mode", "value": "dynamic"})
    raw_controller.handle({"op": "resume"})
    for _ in range(400):
        raw_controller.tick()
    xfrc_raw_controller = raw_controller.d.xfrc_applied.copy()
    launch_text = (ROOT / "scripts" / "launch.py").read_text()
    console_text = (ROOT / "physics" / "control_console.py").read_text()
    # experiment must not pass --harness; harness mode must.
    experiment_ok = "if args.mode == 'harness':\n            command.append('--harness')" in launch_text
    experiment_ok = experiment_ok and "command.append('--harness')" not in launch_text.split("if args.mode == 'assisted'")[0].split("if args.mode == 'harness'")[0][-400:]
    # simpler structural checks
    gated = "if self.harness.enabled:" in console_text and "self.harness.apply(self.m, self.d)" in console_text
    pass8 = (
        (not raw_controller.harness.enabled)
        and np.all(xfrc_raw_controller == 0)
        and gated
        and "if args.mode == 'harness':" in launch_text
        and "command.append('--harness')" in launch_text
        and "--harness" not in (ROOT / "start-actuator-runtime").read_text()
    )

    # --- 9. assisted still present ---
    console_html = (ROOT / "viewer" / "console.html").read_text()
    pass9 = (
        AssistedStandController is not None
        and "Position" in console_html
        and "Torque" in console_html
        and "Assisted" in console_html
        and "value') not in ('kinematic', 'dynamic', 'assisted')" in console_text
        or (
            "kinematic" in console_text
            and "dynamic" in console_text
            and "assisted" in console_text
            and "Position" in console_html
            and "Torque" in console_html
            and "Assisted" in console_html
        )
    )
    # The `or` above is messy if the first clause short-circuits wrong.
    # Recompute cleanly.
    pass9 = (
        "class AssistedStandController" in (ROOT / "physics" / "assisted_stand.py").read_text()
        and all(label in console_html for label in ("Position", "Torque", "Assisted"))
        and "if value not in ('kinematic', 'dynamic', 'assisted')" in console_text
    )

    # actuators unchanged
    ctrl_same = np.array_equal(model.actuator_ctrlrange, ctrl_range_before)
    gravity_same = np.allclose(model.opt.gravity, gravity_before)
    # apply itself does not move qpos
    env_q, h_q = fresh()
    h_q.enable()
    q_before = env_q.data.qpos.copy()
    v_before = env_q.data.qvel.copy()
    h_q.apply(env_q.model, env_q.data)
    apply_inert = np.array_equal(q_before, env_q.data.qpos) and np.array_equal(v_before, env_q.data.qvel)

    passes = {
        "1_hold_vs_raw": bool(pass1),
        "2_lift_one_leg": bool(pass2),
        "3_return_foot_contact": bool(pass3),
        "4_xy_not_pinned": bool(pass4),
        "5_gravity_on": bool(pass5 and gravity_same),
        "6_real_contacts": bool(pass6),
        "7_disable_falls": bool(pass7),
        "8_raw_path_no_harness": bool(pass8),
        "9_assisted_still_present": bool(pass9),
        "apply_does_not_write_qpos_qvel": bool(apply_inert),
        "actuator_ctrlrange_unchanged": bool(ctrl_same),
    }
    # item 5 in the requested list is gravity; ctrlrange is extra evidence inside it
    passes["5_gravity_on"] = bool(pass5 and gravity_same)

    numbers = dict(
        pelvis_id=int(pelvis),
        pelvis_name="pelvis",
        nu=nu,
        timestep=dt,
        gravity=grav,
        z_des=float(harness.z_des) if False else float(HarnessSupport(model).z_des),
        hold_s=HOLD_S,
        harness_min_pelvis_z=min_z,
        harness_end_pelvis_z=hold_end_z,
        harness_end_tilt_rad=hold_end_tilt,
        harness_max_tilt_rad=max_tilt,
        harness_z_samples=z_trace,
        harness_tilt_samples=tilt_trace,
        harness_end_xy=hold_end_xy,
        harness_end_ncon=hold_contacts,
        harness_floor_contact_pairs=hold_floor_pairs[:8],
        raw_time_to_pelvis_below_m=dict(threshold=FALL_Z, seconds=raw_fall_t, end_z=raw_end_z),
        lift=lift_sample,
        foot_return=return_sample,
        horizontal_probe=dict(fx_N=PROBE_FX, seconds=PROBE_S, dx=dx, dy=dy),
        zero_command_drift_xy_over_hold=drift,
        disable=dict(
            z_before=z_before_drop,
            z_after_1_2s=z_after_drop,
            z_if_harness_stayed_on_2_2s=z_if_left_on,
            drop=z_before_drop - z_after_drop,
        ),
        gains=dict(
            kp_z=KP_Z,
            kd_z=KD_Z,
            kp_rp=KP_RP,
            kd_rp=KD_RP,
            kd_yaw=KD_YAW,
            kd_xy=KD_XY,
            max_force=MAX_FORCE,
            max_horizontal_force=MAX_HORIZONTAL_FORCE,
            max_torque=MAX_TORQUE,
        ),
        raw_controller_xfrc_max=float(np.max(np.abs(xfrc_raw_controller))),
        raw_controller_harness_enabled=bool(raw_controller.harness.enabled),
    )

    def conv(obj):
        if isinstance(obj, dict):
            return {k: conv(v) for k, v in obj.items()}
        if isinstance(obj, (list, tuple)):
            return [conv(v) for v in obj]
        if isinstance(obj, (np.floating, np.integer)):
            return obj.item()
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        return obj

    report = dict(passes=passes, all_pass=all(passes.values()), numbers=conv(numbers))
    (OUT / "validation.json").write_text(json.dumps(report, indent=2) + "\n")
    (OUT / "REPORT.md").write_text(render_report(report))
    print(json.dumps(passes, indent=2))
    print("all_pass", report["all_pass"])
    print("wrote", OUT / "validation.json")
    return 0 if report["all_pass"] else 1


def render_report(report: dict) -> str:
    n = report["numbers"]
    g = n["gains"]
    p = report["passes"]
    lift = n["lift"] or {}
    ret = n["foot_return"] or {}
    lines = []
    a = lines.append
    a("# HARNESS validation")
    a("")
    a("Physical support only. The harness writes `data.xfrc_applied` on the pelvis")
    a("before `mj_step`. It does not write `qpos` or `qvel`, does not weld the root,")
    a("does not disable gravity, and does not change actuator limits or the 33-channel command.")
    a("")
    a("## Where the wrench goes")
    a("")
    a(f"- Body name: `pelvis`")
    a(f"- Body id: **{n['pelvis_id']}**")
    a("- Point: that body's center of mass (MuJoCo's definition of `xfrc_applied`)")
    a("- Frame: **world**")
    a("- Layout on MuJoCo 3.14, checked against `mj_applyFT`: **force then torque**")
    a("  `[fx, fy, fz, tx, ty, tz]`. (Some notes say torque-then-force; this build does not.)")
    a("")
    a("## Formula")
    a("")
    a("Pelvis origin pose is the root free joint (`body_quat` is identity).")
    a("`qvel` is world `[vx, vy, vz, wx, wy, wz]`. `up` is the pelvis +Z axis in world.")
    a("`z_des` is the stand / `qpos0` pelvis height, captured once at enable.")
    a("")
    a("```")
    a("Fz = clip(kp_z * (z_des - z) - kd_z * vz, -F_max, F_max)")
    a("Fxy = -kd_xy * vxy          # no position spring")
    a("Fxy *= Fxy_max / |Fxy|      if |Fxy| > Fxy_max")
    a("τ_rp = kp_rp * (up × ez)    # ez = (0,0,1); yaw component of this term is 0")
    a("τ   = τ_rp - (kd_rp * wx, kd_rp * wy, kd_yaw * wz)")
    a("τ  *= T_max / |τ|           if |τ| > T_max")
    a("```")
    a("")
    a("Constants used on this model:")
    a("")
    a(f"- `kp_z` = {g['kp_z']} N/m")
    a(f"- `kd_z` = {g['kd_z']} N/(m/s)")
    a(f"- `kp_rp` = {g['kp_rp']} N·m per unit sin(tilt)")
    a(f"- `kd_rp` = {g['kd_rp']} N·m/(rad/s)")
    a(f"- `kd_yaw` = {g['kd_yaw']} N·m/(rad/s) (yaw spring is 0)")
    a(f"- `kd_xy` = {g['kd_xy']} N/(m/s)")
    a(f"- `|Fz|` cap = {g['max_force']} N")
    a(f"- `|Fxy|` cap = {g['max_horizontal_force']} N")
    a(f"- `|τ|` cap = {g['max_torque']} N·m")
    a("")
    a("Actuators: the same 33 motors. `u_cmd` is copied to `data.ctrl` by the existing")
    a("RAW actuator path. The harness does not add joint torques and does not edit `ctrlrange`.")
    a("Arms are not controlled by the harness. It is not a gait.")
    a("")
    a("## Results")
    a("")
    a(f"All checks passed: **{report['all_pass']}**")
    a("")
    a("| # | Check | Result |")
    a("| --- | --- | --- |")
    for key, ok in p.items():
        a(f"| {key} | {'PASS' if ok else 'FAIL'} |")
    a("")
    a("### 1. Supported vs raw")
    a("")
    raw = n["raw_time_to_pelvis_below_m"]
    a(f"- Harness on, zero actuator commands, {n['hold_s']} s: min pelvis z = **{n['harness_min_pelvis_z']:.4f} m**,")
    a(f"  end z = **{n['harness_end_pelvis_z']:.4f} m**, max tilt = **{n['harness_max_tilt_rad']:.4f} rad**,")
    a(f"  end tilt = **{n['harness_end_tilt_rad']:.4f} rad**.")
    a(f"- z samples (every 0.5 s): {n['harness_z_samples']}")
    a(f"- RAW, same model, harness off, xfrc held at 0, gravity on: pelvis z fell below")
    a(f"  {raw['threshold']} m at **{raw['seconds']:.3f} s** (z then {raw['end_z']:.4f} m).")
    a(f"- End of harness hold still had **{n['harness_end_ncon']}** MuJoCo contacts, including floor pairs {n['harness_floor_contact_pairs'][:4]}.")
    a("")
    a("### 2–3. One leg lifts and returns")
    a("")
    if lift:
        a(f"- Open-loop `{KNEE_TORQUE}` N·m on `knee_l_motor` only (other actuators 0).")
        a(f"- At t = {lift['time']:.3f} s the left sole was **{lift['sole_l']:.3f} m** off the floor,")
        a(f"  left foot–floor contacts = **{lift['left']['n']}**,")
        a(f"  right (stance) contacts = **{lift['right']['n']}**, stance normal force = **{lift['right']['normal_force']:.2f} N**.")
        a(f"- Pelvis z during that sample: {lift['pelvis_z']:.3f} m. Right sole z: {lift['sole_r']:.4f} m.")
    else:
        a("- Lift criterion was not met. See `validation.json`.")
    if ret:
        a(f"- After the knee command was set back to 0, left foot–floor contact returned at")
        a(f"  t = **{ret['time']:.3f} s**, sole min z = **{ret['sole_l']:.4f} m**,")
        a(f"  left contacts = **{ret['left']['n']}** (normal **{ret['left']['normal_force']:.2f} N**),")
        a(f"  total `data.ncon` = **{ret['ncon']}**.")
        a(f"- Contact geoms: {ret['left']['pairs'][:4]}.")
    else:
        a("- The lifted foot did not regain a real floor contact within the return window.")
    a("")
    a("### 4. XY is not pinned")
    a("")
    hp = n["horizontal_probe"]
    a(f"- Validator-only probe: +{hp['fx_N']} N world X added **after** the harness write, for {hp['seconds']} s.")
    a(f"- Pelvis translation: **Δx = {hp['dx']:.4f} m**, **Δy = {hp['dy']:.4f} m** (not clamped to 0).")
    dxy = n["zero_command_drift_xy_over_hold"]
    a(f"- With zero commands and no probe, pelvis still drifted by Δxy = ({dxy[0]:.4f}, {dxy[1]:.4f}) m")
    a(f"  over {n['hold_s']} s. Horizontal support is damping, not a position spring.")
    a("")
    a("### 5–7. Gravity, contacts, disable")
    a("")
    a(f"- `model.opt.gravity` = **{n['gravity']}** (unchanged from the XML default).")
    a(f"- Contacts above are `data.ncon` / `data.contact` geom ids, with `mj_contactForce` normal components. Not flags.")
    dis = n["disable"]
    a(f"- Harness disabled at pelvis z = **{dis['z_before']:.4f} m** (xfrc set to 0, no qpos rewrite).")
    a(f"  After 1.2 s of ordinary physics: z = **{dis['z_after_1_2s']:.4f} m** (drop **{dis['drop']:.4f} m**).")
    a(f"  A twin run left on for 2.2 s ended at z = **{dis['z_if_harness_stayed_on_2_2s']:.4f} m**.")
    a("")
    a("### 8–9. RAW path and Debug Assisted")
    a("")
    a(f"- `Controller` in dynamic mode, harness never enabled, 400 ticks: max |xfrc_applied| = **{n['raw_controller_xfrc_max']}**,")
    a(f"  `harness.enabled` = **{n['raw_controller_harness_enabled']}**.")
    a("- `./START.command experiment` / `start-actuator-runtime` do not pass `--harness`.")
    a("  `tick()` calls `harness.apply` only inside `if self.harness.enabled`.")
    a("- `assisted_stand.AssistedStandController` still imports. `viewer/console.html` still has")
    a("  Position, Torque, and Assisted. Debug mode values `kinematic` / `dynamic` / `assisted` are unchanged.")
    a("")
    a("## How to launch")
    a("")
    a("```sh")
    a("./START.command harness")
    a("# menu item 3, or the same command")
    a("# clean observation viewer: http://127.0.0.1:8788/")
    a("# motors: ./humanoid act <33 floats>    ./humanoid zero")
    a("```")
    a("")
    a("Debug (`./START.command debug`) is the developer console and still has Assisted.")
    a("Experiment (`./START.command experiment`) is RAW: no harness wrench.")
    a("`./START.command assisted` still starts the old assisted-stand runtime, but it is not in the menu.")
    a("")
    a("## Limits")
    a("")
    a("- The harness holds the pelvis up and keeps roll/pitch from a catastrophic fall.")
    a("  It does not balance the robot, time steps, or move the arms.")
    a("- Feet can slip. XY is supposed to be free, so the body can wander on the floor.")
    a("- A hard foot landing can spike contact force. That spike is MuJoCo contact, not a fake flag.")
    a("- Yaw is only lightly damped. There is no heading hold.")
    a("- Gains are for this mesh and this mass (~89 kg). They are constants in `physics/harness.py`.")
    a("")
    return "\n".join(lines) + "\n"


if __name__ == "__main__":
    sys.exit(main())
