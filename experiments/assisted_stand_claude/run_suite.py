"""Headless assisted-stand investigation suite. Simulated time only, compact summaries.

  .venv/bin/python experiments/assisted_stand_claude/run_suite.py [--quick]

Writes experiments/assisted_stand_claude/results.json.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import mujoco
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from stand_lib import (ROOT, StanceID, actuator_dofs, equilibrium_seed, floor_fz, load_model,  # noqa: E402
                       pitch_back_deg, tilt_deg)

sys.path.insert(0, str(ROOT / "physics"))
from assisted_stand import AssistedStandController, _class_of  # noqa: E402

QUICK = "--quick" in sys.argv
R: dict = {}


def diagnostics():
    m = load_model("mesh")
    d = mujoco.MjData(m)
    d.qpos[:] = m.qpos0
    mujoco.mj_forward(m, d)
    g0 = d.qfrc_bias.copy()
    M = np.zeros((m.nv, m.nv))
    mujoco.mj_fullM(m, d, M)
    rows = []
    for i, dof in enumerate(actuator_dofs(m)):
        name = m.actuator(i).name
        dq = np.zeros(m.nv)
        dq[dof] = 1e-4
        q = m.qpos0.copy()
        mujoco.mj_integratePos(m, q, dq, 1.0)
        d.qpos[:] = q
        mujoco.mj_forward(m, d)
        # destabilizing gravity stiffness: -(d bias / dq); > kp means the joint PD cannot hold even statically
        k_grav = -(d.qfrc_bias[dof] - g0[dof]) / 1e-4
        kp, kd = AssistedStandController.DEFAULT_GAINS[_class_of(name)]
        rows.append(dict(actuator=name, hold_torque=round(float(g0[dof]), 3), k_grav=round(float(k_grav), 2),
                         kp=round(kp, 2), kd=round(kd, 3), inertia=round(float(M[dof, dof]), 5),
                         kd_dt_over_I=round(kd * m.opt.timestep / M[dof, dof], 3)))
    d.qpos[:] = m.qpos0
    mujoco.mj_forward(m, d)
    jc = np.zeros((3, m.nv))
    mujoco.mj_jacSubtreeCom(m, d, jc, 1)
    jcol = {n: float(np.linalg.norm(jc[:, m.jnt_dofadr[m.joint(n).id]])) for n in ("ankle_l_dp", "knee_l", "hip_l", "lumbar")}
    fz0 = floor_fz(m, d)
    lift = equilibrium_seed(m, d)
    feet = {"foot_l_geom": 0, "foot_r_geom": 0}
    for i in range(d.ncon):
        for g in (d.contact[i].geom1, d.contact[i].geom2):
            n = m.geom(g).name
            if n in feet:
                feet[n] += 1
    R["diagnostics"] = dict(
        total_mass=float(m.body_subtreemass[1]), joints=rows, com_jacobian_column_norm_m_per_rad=jcol,
        floor_force_at_qpos0_N=round(fz0, 1), weight_N=round(float(m.body_subtreemass[1]) * 9.81, 1),
        foot_geom_margin_m=float(m.geom_margin[m.geom("foot_l_geom").id]),
        equilibrium_seed_lift_mm=round(lift * 1000, 2), floor_contacts_per_foot_at_seed=feet)
    print("== diagnostics")
    for r in rows:
        if any(k in r["actuator"] for k in ("lumbar_x", "thoracic_x", "neck_x", "ankle_l", "wrist_l_dev")):
            print("  ", r)
    print("   COM-Jacobian column norms:", {k: round(v, 4) for k, v in jcol.items()})
    print(f"   floor force at qpos0 {fz0:.0f} N vs weight {m.body_subtreemass[1] * 9.81:.0f} N; equilibrium lift {lift * 1000:.2f} mm;"
          f" contacts at seed {feet}")


def run_assisted(variant, T):
    """Existing AssistedStandController exactly as the console runs it (reset_to_stand, compute, mj_step)."""
    m = load_model(variant)
    d = mujoco.MjData(m)
    c = AssistedStandController(m)
    if variant == "flat_sole":
        c._foot_geoms = {m.geom("sole_l").id, m.geom("sole_r").id}
    c.reset_to_stand(m, d)
    c.enabled = True
    tu, head = m.body("torso_upper").id, m.body("head").id
    old_ok_until = None  # previous validator: pelvis z >= 0.70 and pelvis tilt <= 0.55 rad
    torso_ok_until = None  # plus torso_upper tilt <= 20 deg
    mx_torso, mx_head = 0.0, 0.0
    for k in range(int(T / m.opt.timestep)):
        d.ctrl[:] = c.compute(m, d, np.zeros(m.nu))
        mujoco.mj_step(m, d)
        mujoco.mj_forward(m, d)
        mx_torso = max(mx_torso, abs(pitch_back_deg(d, tu)))
        mx_head = max(mx_head, abs(pitch_back_deg(d, head)))
        up = d.qpos[2] >= 0.70 and np.radians(tilt_deg(d, 1)) <= 0.55
        if old_ok_until is None and not up:
            old_ok_until = d.time
        if torso_ok_until is None and (not up or tilt_deg(d, tu) > 20):
            torso_ok_until = d.time
    return dict(variant=variant, sim_s=T, old_criteria_fail_at_s=old_ok_until, torso_criteria_fail_at_s=torso_ok_until,
                max_torso_pitch_deg=round(mx_torso, 1), max_head_pitch_deg=round(mx_head, 1))


def run_stance(variant, T, cop, push=None, noise=0.0, seed=0):
    m = load_model(variant)
    d = mujoco.MjData(m)
    equilibrium_seed(m, d)
    c = StanceID(m, cop=cop)
    c.reset(d)
    if noise:
        dq = np.random.default_rng(seed).normal(0, noise, m.nv)
        dq[:6] = 0
        mujoco.mj_integratePos(m, d.qpos, dq, 1.0)
        mujoco.mj_forward(m, d)
    tu = m.body("torso_upper").id
    fell, mx_p, mx_t, sat = None, 0.0, 0.0, 0
    for k in range(int(T / m.opt.timestep)):
        d.ctrl[:] = c.compute(m, d)
        d.xfrc_applied[:] = 0
        if push is not None and push[0] <= d.time < push[0] + 0.1:  # test disturbance only
            d.xfrc_applied[tu, :3] = push[1]
        mujoco.mj_step(m, d)
        if not np.isfinite(d.qpos).all():
            return dict(error="nonfinite", t=d.time)
        mx_p, mx_t = max(mx_p, tilt_deg(d, 1)), max(mx_t, tilt_deg(d, tu))
        sat = max(sat, c.diag["saturated"])
        if fell is None and (d.qpos[2] < 0.75 or tilt_deg(d, tu) > 30):
            fell = round(d.time, 3)
    return dict(fell_at_s=fell, max_pelvis_tilt_deg=round(mx_p, 2), max_torso_tilt_deg=round(mx_t, 2), max_saturated=sat)


def main():
    diagnostics()
    T = 20.0 if QUICK else 120.0
    print("== existing Assisted controller")
    R["assisted_existing"] = [run_assisted("mesh", 15.0), run_assisted("flat_sole", 15.0)]
    for r in R["assisted_existing"]:
        print("  ", r)
    print(f"== StanceID, undisturbed hold {T:.0f} s")
    R["stance_hold"] = {}
    for variant, cop in (("mesh", "free"), ("flat_sole", "free"), ("flat_sole", "corners")):
        r = run_stance(variant, T, cop)
        R["stance_hold"][f"{variant}/{cop}"] = r
        print(f"   {variant}/{cop}:", r)
    print("== StanceID flat_sole/corners: 0.1 s pushes at torso_upper, then 5 s")
    R["pushes"] = {}
    for label, v in (("fwd", (0, -1, 0)), ("back", (0, 1, 0)), ("side", (1, 0, 0))):
        for n in (50, 100, 200):
            r = run_stance("flat_sole", 6.0, "corners", push=(1.0, np.array(v, float) * n))
            R["pushes"][f"{label}_{n * 0.1:.0f}Ns"] = r["fell_at_s"]
    print("  ", R["pushes"])
    R["noisy_start_0.05rad"] = [run_stance("flat_sole", 10.0, "corners", noise=0.05, seed=s)["fell_at_s"] for s in range(5)]
    print("   noisy start sigma 0.05 rad, fell_at_s per seed:", R["noisy_start_0.05rad"])
    out = HERE / "results.json"
    out.write_text(json.dumps(R, indent=1))
    print("wrote", out.relative_to(ROOT))


if __name__ == "__main__":
    main()
