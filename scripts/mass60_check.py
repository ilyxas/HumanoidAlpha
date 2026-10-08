#!/usr/bin/env python3
"""Before/after compiled mass checks and A/B behavior for the 60 kg experiment.

Baseline is always `git show main:physics/model/humanoid_alpha.xml` (the
~89 kg model). The experimental model is physics/model/humanoid_alpha.xml
on this branch. No GUI. Does not retune controllers.

Run from the repo root:
  .venv/bin/python scripts/mass60_check.py
"""
from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "physics"))

from harness import HarnessSupport, KP_Z  # noqa: E402

OUT = ROOT / "reports" / "mass60"
MODEL = ROOT / "physics" / "model" / "humanoid_alpha.xml"
ASSUMPTIONS = ROOT / "physics" / "assumptions.json"
BEFORE = ROOT / "reports" / "collision" / "model_before.xml"
MASS_ATOL = 1e-8
INERTIA_ATOL = 1e-9


def load_main_baseline() -> mujoco.MjModel:
    blob = subprocess.check_output(
        ["git", "show", "main:physics/model/humanoid_alpha.xml"], cwd=ROOT
    )
    # Mesh paths are relative to the MJCF file, so the temp copy must sit
    # next to the real model. Removed before return.
    tmp = ROOT / "physics" / "model" / "_tmp_main_baseline_mass60.xml"
    tmp.write_bytes(blob)
    try:
        return mujoco.MjModel.from_xml_path(str(tmp))
    finally:
        tmp.unlink(missing_ok=True)


def body_rows(model: mujoco.MjModel) -> list[dict]:
    rows = []
    for i in range(model.nbody):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i)
        if not name or name == "world":
            continue
        I = np.asarray(model.body_inertia[i], dtype=float)
        rows.append(
            dict(
                body=name,
                mass=float(model.body_mass[i]),
                ipos=np.asarray(model.body_ipos[i], dtype=float).tolist(),
                iquat=np.asarray(model.body_iquat[i], dtype=float).tolist(),
                inertia=I.tolist(),
            )
        )
    return rows


def triangle_ok(I: np.ndarray) -> bool:
    a, b, c = (float(x) for x in I)
    eps = 1e-12
    return (
        np.all(np.isfinite(I))
        and a > 0
        and b > 0
        and c > 0
        and a + b >= c - eps
        and a + c >= b - eps
        and b + c >= a - eps
    )


def names(model, obj) -> list[str]:
    return [mujoco.mj_id2name(model, obj, i) for i in range(int(getattr(model, {
        mujoco.mjtObj.mjOBJ_ACTUATOR: "nu",
        mujoco.mjtObj.mjOBJ_JOINT: "njnt",
        mujoco.mjtObj.mjOBJ_GEOM: "ngeom",
    }[obj])))]


def tilt_of(quat) -> float:
    w, x, y, z = (float(v) for v in quat)
    up_z = 1.0 - 2.0 * (x * x + y * y)
    return float(np.arccos(np.clip(up_z, -1.0, 1.0)))


def ball_angle(q) -> float:
    w = abs(float(q[0]))
    return float(2.0 * np.arccos(np.clip(w, 0.0, 1.0)))


def sole_min_z(model, data, geom_name: str) -> float:
    gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, geom_name)
    mid = int(model.geom_dataid[gid])
    adr = int(model.mesh_vertadr[mid])
    n = int(model.mesh_vertnum[mid])
    verts = model.mesh_vert[adr : adr + n]
    rot = data.geom_xmat[gid].reshape(3, 3)
    world = verts @ rot.T + data.geom_xpos[gid]
    return float(world[:, 2].min())


def foot_contacts(model, data, foot_geom: str) -> int:
    gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, foot_geom)
    floor = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "floor")
    n = 0
    for i in range(data.ncon):
        c = data.contact[i]
        pair = {int(c.geom1), int(c.geom2)}
        if gid in pair and floor in pair:
            n += 1
    return n


def reset(model) -> mujoco.MjData:
    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)
    data.qpos[:] = model.qpos0
    data.qvel[:] = 0
    data.ctrl[:] = 0
    data.xfrc_applied[:] = 0
    mujoco.mj_forward(model, data)
    return data


def sample_times(seconds: float, step: float) -> list[float]:
    n = int(round(seconds / step))
    times = [round(i * step, 10) for i in range(0, n + 1)]
    return times


def simulate(model, seconds: float, setup, record_at: list[float]) -> dict:
    data = reset(model)
    setup(model, data, None)  # initial condition hook
    dt = float(model.opt.timestep)
    n = int(round(seconds / dt))
    want_steps = {int(round(t / dt)) for t in record_at}
    want_steps.add(0)
    want_steps.add(n)
    samples = []
    fall_t = None
    min_z = float("inf")
    max_tilt = 0.0
    for i in range(n + 1):
        hook = getattr(setup, "before_step", None)
        if hook:
            hook(model, data)
        t = float(data.time)
        z = float(data.qpos[2])
        tilt = tilt_of(data.qpos[3:7])
        min_z = min(min_z, z)
        max_tilt = max(max_tilt, tilt)
        if fall_t is None and z < 0.55:
            fall_t = t
        if i in want_steps:
            extra = {}
            setup_extra = getattr(setup, "observe", None)
            if setup_extra:
                extra = setup_extra(model, data)
            samples.append(dict(t=t, pelvis_z=z, tilt=tilt, **extra))
        if i == n:
            break
        mujoco.mj_step(model, data)
    return dict(samples=samples, fall_t=fall_t, min_z=min_z, max_tilt=max_tilt, end_t=float(data.time))


class Runner:
    def __init__(self, before_step=None, observe=None, init=None):
        self.before_step = before_step
        self.observe = observe
        self.init = init

    def __call__(self, model, data, _):
        if self.init:
            self.init(model, data)


def zero_control(model, seconds=2.0):
    marks = [0.0, 0.5, 1.0, 1.5, 2.0]

    def before(model, data):
        data.ctrl[:] = 0
        data.xfrc_applied[:] = 0

    run = Runner(before_step=before)
    return simulate(model, seconds, run, marks)


def harness_hold(model, seconds=5.0):
    h = HarnessSupport(model)
    marks = [0.0, 1.0, 2.0, 3.0, 4.0, 5.0]
    pid = h.pelvis_id

    def init(model, data):
        h.seed_stand(model, data)
        h.enable()
        data.ctrl[:] = 0

    def before(model, data):
        data.ctrl[:] = 0
        h.apply(model, data)

    def observe(model, data):
        w = np.asarray(data.xfrc_applied[pid], dtype=float)
        return dict(
            wrench_norm=float(np.linalg.norm(w)),
            force_norm=float(np.linalg.norm(w[:3])),
            torque_norm=float(np.linalg.norm(w[3:])),
            fz=float(w[2]),
        )

    run = Runner(before_step=before, observe=observe, init=init)
    out = simulate(model, seconds, run, marks)
    out["z_des"] = float(h.z_des)
    out["pelvis_id_name"] = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, pid)
    # static spring sag if Fz = weight and velocity is ~0: mg/KP_Z
    mass = float(np.sum(model.body_mass))
    weight = mass * abs(float(model.opt.gravity[2]))
    out["weight_N"] = weight
    out["predicted_static_sag_m"] = weight / KP_Z
    zs = [s["pelvis_z"] for s in out["samples"]]
    out["end_sag_m"] = float(h.z_des - zs[-1])
    out["holds_validator_criterion"] = bool(out["min_z"] > 0.75 and out["max_tilt"] < 0.6)
    return out


def torque_test(model, seconds, ctrl_map: dict, with_harness: bool, observe_fn):
    h = HarnessSupport(model) if with_harness else None
    marks = [round(t, 6) for t in np.arange(0.0, seconds + 1e-9, 0.2)]

    def init(model, data):
        if h is not None:
            h.seed_stand(model, data)
            h.enable()
        data.ctrl[:] = 0
        for name, value in ctrl_map.items():
            aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
            data.ctrl[aid] = value

    def before(model, data):
        if h is not None:
            h.apply(model, data)
        else:
            data.xfrc_applied[:] = 0
        for name, value in ctrl_map.items():
            aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
            data.ctrl[aid] = value

    run = Runner(before_step=before, observe=observe_fn, init=init)
    return simulate(model, seconds, run, marks)


def lumbar_thoracic(model):
    def observe(model, data):
        out = {}
        for joint in ("lumbar", "thoracic"):
            j = model.joint(joint)
            adr = int(j.qposadr[0])
            out[joint + "_angle_rad"] = ball_angle(data.qpos[adr : adr + 4])
        return out

    return torque_test(
        model,
        0.8,
        {"lumbar_x_motor": 20.0, "thoracic_x_motor": 15.0},
        with_harness=False,
        observe_fn=observe,
    )


def knee_test(model):
    def observe(model, data):
        j = model.joint("knee_l")
        adr = int(j.qposadr[0])
        return dict(
            knee_l_rad=float(data.qpos[adr]),
            sole_l_z=sole_min_z(model, data, "foot_l_geom"),
            sole_r_z=sole_min_z(model, data, "foot_r_geom"),
            foot_l_floor_contacts=foot_contacts(model, data, "foot_l_geom"),
            foot_r_floor_contacts=foot_contacts(model, data, "foot_r_geom"),
        )

    out = torque_test(
        model,
        0.8,
        {"knee_l_motor": 70.0},
        with_harness=True,
        observe_fn=observe,
    )
    end = out["samples"][-1]
    out["lifted"] = bool(
        end["foot_l_floor_contacts"] == 0 and end["sole_l_z"] > 0.15 and end["foot_r_floor_contacts"] > 0
    )
    return out


def geom_fingerprint(model: mujoco.MjModel) -> dict:
    rows = []
    for i in range(model.ngeom):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i)
        rows.append(
            (
                name,
                int(model.geom_type[i]),
                np.round(model.geom_size[i], 12).tolist(),
                np.round(model.geom_pos[i], 12).tolist(),
                int(model.geom_contype[i]),
                int(model.geom_conaffinity[i]),
                int(model.geom_group[i]),
            )
        )
    return dict(ngeom=model.ngeom, rows=rows)


def check_invariants(base: mujoco.MjModel, exp: mujoco.MjModel) -> list[dict]:
    checks = []

    def add(name, ok, detail):
        checks.append(dict(name=name, pass_=bool(ok), detail=detail))

    add("nq", base.nq == exp.nq, f"{base.nq} -> {exp.nq}")
    add("nv", base.nv == exp.nv, f"{base.nv} -> {exp.nv}")
    add("nu", base.nu == exp.nu, f"{base.nu} -> {exp.nu}")
    b_act = [base.actuator(i).name for i in range(base.nu)]
    e_act = [exp.actuator(i).name for i in range(exp.nu)]
    add("actuator_names_order", b_act == e_act, f"n={len(e_act)} first={e_act[0]} last={e_act[-1]}")
    add(
        "ctrlrange",
        np.allclose(base.actuator_ctrlrange, exp.actuator_ctrlrange),
        "max abs diff "
        + str(float(np.max(np.abs(base.actuator_ctrlrange - exp.actuator_ctrlrange)))),
    )
    add(
        "jnt_range",
        np.allclose(base.jnt_range, exp.jnt_range),
        "max abs diff " + str(float(np.max(np.abs(base.jnt_range - exp.jnt_range)))),
    )
    add(
        "jnt_axis",
        np.allclose(base.jnt_axis, exp.jnt_axis),
        "max abs diff " + str(float(np.max(np.abs(base.jnt_axis - exp.jnt_axis)))),
    )
    add(
        "qpos0",
        np.allclose(base.qpos0, exp.qpos0),
        "max abs diff " + str(float(np.max(np.abs(base.qpos0 - exp.qpos0)))),
    )
    add(
        "gravity",
        np.array_equal(base.opt.gravity, exp.opt.gravity),
        str(np.asarray(exp.opt.gravity).tolist()),
    )
    add(
        "timestep",
        base.opt.timestep == exp.opt.timestep,
        str(float(exp.opt.timestep)),
    )
    add(
        "body_ipos",
        np.allclose(base.body_ipos, exp.body_ipos, atol=1e-12),
        "max abs diff " + str(float(np.max(np.abs(base.body_ipos - exp.body_ipos)))),
    )
    add(
        "body_iquat",
        np.allclose(base.body_iquat, exp.body_iquat, atol=1e-12),
        "max abs diff " + str(float(np.max(np.abs(base.body_iquat - exp.body_iquat)))),
    )
    # radii of gyration preserved: I/m
    with np.errstate(divide="ignore", invalid="ignore"):
        kb = base.body_inertia / base.body_mass[:, None]
        ke = exp.body_inertia / exp.body_mass[:, None]
    # world body mass 0
    mask = base.body_mass > 0
    add(
        "radii_of_gyration_I_over_m",
        np.allclose(kb[mask], ke[mask], rtol=1e-9, atol=1e-12),
        "max abs diff " + str(float(np.max(np.abs(kb[mask] - ke[mask])))),
    )
    gb, ge = geom_fingerprint(base), geom_fingerprint(exp)
    add("collision_geom_fingerprint", gb == ge, f"ngeom {gb['ngeom']}")
    add(
        "dof_damping",
        np.allclose(base.dof_damping, exp.dof_damping),
        "max abs diff " + str(float(np.max(np.abs(base.dof_damping - exp.dof_damping)))),
    )
    add(
        "dof_armature",
        np.allclose(base.dof_armature, exp.dof_armature),
        "max abs diff " + str(float(np.max(np.abs(base.dof_armature - exp.dof_armature)))),
    )
    return checks


def mass_table(base_rows, intended) -> list[dict]:
    rows = []
    for r in base_rows:
        m_i = float(intended[r["body"]])
        rows.append(
            dict(
                body=r["body"],
                current_mass=r["mass"],
                intended_mass=m_i,
                ratio=r["mass"] / m_i,
                inertia_valid=triangle_ok(np.asarray(r["inertia"])),
            )
        )
    return rows


def fmt(x, n=6):
    if x is None:
        return "None"
    return f"{x:.{n}f}"


def write_report(payload: dict) -> str:
    lines = []
    a = lines.append
    a("# Mass model: 89 kg baseline vs per-body 60 kg")
    a("")
    a(f"- Branch: `{payload['branch']}`")
    a(f"- main (untouched parent): `{payload['head']}` at generation time. The experiment commit is the commit that adds this file on `exp/mass-60kg-per-body`; it is not main.")
    a(f"- Baseline loaded from: `git show main:physics/model/humanoid_alpha.xml`")
    a(f"- Experimental model: `physics/model/humanoid_alpha.xml`")
    a("")
    a("## Inertia method")
    a("")
    a(payload["method"])
    a("")
    a("## Before table (compiled baseline vs assumptions.json masses_kg)")
    a("")
    a("| body | current mass kg | intended mass kg | ratio current/intended | baseline inertia valid |")
    a("|---|---:|---:|---:|:---:|")
    for r in payload["before_table"]:
        a(
            f"| {r['body']} | {r['current_mass']:.8f} | {r['intended_mass']:.8f} | {r['ratio']:.6f} | {r['inertia_valid']} |"
        )
    bt = payload["before_total"]
    a(f"| **TOTAL** | **{bt['current']:.8f}** | **{bt['intended']:.8f}** | **{bt['ratio']:.6f}** | |")
    a("")
    a("## After table (compiled experimental model)")
    a("")
    a("| body | compiled mass kg | intended kg | abs err | inertia valid |")
    a("|---|---:|---:|---:|:---:|")
    for r in payload["after_table"]:
        a(
            f"| {r['body']} | {r['compiled_mass']:.8f} | {r['intended_mass']:.8f} | {r['abs_err']:.3e} | {r['inertia_valid']} |"
        )
    at = payload["after_total"]
    a(f"| **TOTAL** | **{at['compiled']:.8f}** | **{at['intended']:.8f}** | **{at['abs_err']:.3e}** | |")
    a("")
    a("## Invariant checks (compiled baseline vs compiled experimental)")
    a("")
    a("| check | result | detail |")
    a("|---|---|---|")
    for c in payload["invariants"]:
        a(f"| {c['name']} | {'PASS' if c['pass_'] else 'FAIL'} | {c['detail']} |")
    a("")
    a("## Behavioral A/B")
    a("")
    a("Identical `qpos0`, zero initial velocity. No gain changes. A = main-branch compiled model (~89.0437 kg). B = experimental model.")
    a("")
    a("### Zero-control fall (2.0 s, ctrl=0, harness off)")
    a("")
    a("| t s | A pelvis z | B pelvis z | A tilt rad | B tilt rad |")
    a("|---:|---:|---:|---:|---:|")
    for sa, sb in zip(payload["fall"]["A"]["samples"], payload["fall"]["B"]["samples"]):
        a(f"| {sa['t']:.1f} | {sa['pelvis_z']:.6f} | {sb['pelvis_z']:.6f} | {sa['tilt']:.6f} | {sb['tilt']:.6f} |")
    for label in ("A", "B"):
        f = payload["fall"][label]
        a(f"- {label}: first pelvis z < 0.55 s at t={f['fall_t']}, min z={f['min_z']:.6f}, max tilt={f['max_tilt']:.6f}")
    a("")
    a("### HARNESS, zero human command (5.0 s, gains unchanged)")
    a("")
    a("Existing gains were tuned against ~89 kg (`KP_Z=30000` N/m, comment in `physics/harness.py` cites ~874 N). Not retuned.")
    a("")
    a("| t s | A z | B z | A tilt | B tilt | A wrench | B wrench | A Fz | B Fz |")
    a("|---:|---:|---:|---:|---:|---:|---:|---:|---:|")
    for sa, sb in zip(payload["harness"]["A"]["samples"], payload["harness"]["B"]["samples"]):
        a(
            f"| {sa['t']:.1f} | {sa['pelvis_z']:.6f} | {sb['pelvis_z']:.6f} | {sa['tilt']:.6f} | {sb['tilt']:.6f} | {sa['wrench_norm']:.3f} | {sb['wrench_norm']:.3f} | {sa['fz']:.3f} | {sb['fz']:.3f} |"
        )
    for label in ("A", "B"):
        h = payload["harness"][label]
        a(
            f"- {label}: weight={h['weight_N']:.3f} N, predicted static sag mg/KP_Z={h['predicted_static_sag_m']:.6f} m, "
            f"end sag={h['end_sag_m']:.6f} m, min z={h['min_z']:.6f}, max tilt={h['max_tilt']:.6f}, "
            f"validator hold (min z>0.75 and max tilt<0.6)={h['holds_validator_criterion']}"
        )
    a("")
    a("### Lumbar/thoracic open-loop torque (harness off)")
    a("")
    a("`lumbar_x_motor=20` N·m and `thoracic_x_motor=15` N·m for 0.8 s. Other actuators 0. Angle is the ball-joint rotation magnitude.")
    a("")
    a("| t s | A lumbar | B lumbar | A thoracic | B thoracic | A z | B z |")
    a("|---:|---:|---:|---:|---:|---:|---:|")
    for sa, sb in zip(payload["lumbar"]["A"]["samples"], payload["lumbar"]["B"]["samples"]):
        a(
            f"| {sa['t']:.1f} | {sa['lumbar_angle_rad']:.6f} | {sb['lumbar_angle_rad']:.6f} | {sa['thoracic_angle_rad']:.6f} | {sb['thoracic_angle_rad']:.6f} | {sa['pelvis_z']:.6f} | {sb['pelvis_z']:.6f} |"
        )
    a("")
    a("### Existing limb torque test: knee_l 70 N·m with HARNESS (0.8 s)")
    a("")
    a("Same open-loop knee torque the harness validator uses (`KNEE_TORQUE=70`). Gains unchanged.")
    a("")
    a("| t s | A knee rad | B knee rad | A sole_l z | B sole_l z | A left floor contacts | B left floor contacts | A z | B z |")
    a("|---:|---:|---:|---:|---:|---:|---:|---:|---:|")
    for sa, sb in zip(payload["knee"]["A"]["samples"], payload["knee"]["B"]["samples"]):
        a(
            f"| {sa['t']:.1f} | {sa['knee_l_rad']:.6f} | {sb['knee_l_rad']:.6f} | {sa['sole_l_z']:.6f} | {sb['sole_l_z']:.6f} | {sa['foot_l_floor_contacts']} | {sb['foot_l_floor_contacts']} | {sa['pelvis_z']:.6f} | {sb['pelvis_z']:.6f} |"
        )
    a(f"- A lifted (left foot clear, sole>0.15 m, right foot still in floor contact) at t=0.8: {payload['knee']['A']['lifted']}")
    a(f"- B lifted at t=0.8: {payload['knee']['B']['lifted']}")
    a("")
    a("The 20/15 N·m spine command and the 70 N·m knee command both reach essentially the same joint angle by 0.2 s on A and B (spine angles sit on the ball-cone stops: lumbar limit 0.698 rad, thoracic limit 0.436 rad). That comparison is contact/limit dominated, so it does not show the inertia change. A smaller open-loop probe is below.")
    a("")
    a("### Sub-limit probe (not a retune): lumbar_x=2 N·m, thoracic_x=2 N·m, harness off; knee_l=8 N·m with harness")
    a("")
    a("| t s | A lumbar | B lumbar | A thoracic | B thoracic | A z | B z |")
    a("|---:|---:|---:|---:|---:|---:|---:|")
    for sa, sb in zip(payload["sublimit"]["A"]["spine"]["samples"], payload["sublimit"]["B"]["spine"]["samples"]):
        a(
            f"| {sa['t']:.1f} | {sa['lumbar_angle_rad']:.6f} | {sb['lumbar_angle_rad']:.6f} | {sa['thoracic_angle_rad']:.6f} | {sb['thoracic_angle_rad']:.6f} | {sa['pelvis_z']:.6f} | {sb['pelvis_z']:.6f} |"
        )
    a("")
    a("| t s | A knee rad | B knee rad | A sole_l z | B sole_l z | A pelvis z | B pelvis z |")
    a("|---:|---:|---:|---:|---:|---:|---:|")
    for sa, sb in zip(payload["sublimit"]["A"]["knee"]["samples"], payload["sublimit"]["B"]["knee"]["samples"]):
        a(
            f"| {sa['t']:.1f} | {sa['knee_l_rad']:.6f} | {sb['knee_l_rad']:.6f} | {sa['sole_l_z']:.6f} | {sb['sole_l_z']:.6f} | {sa['pelvis_z']:.6f} | {sb['pelvis_z']:.6f} |"
        )
    a("")
    a("## Provenance")
    a("")
    a(payload["provenance_md"])
    a("")
    a("## Failures / unexpected")
    a("")
    a(payload["failures_md"])
    a("")
    return "\n".join(lines) + "\n"



def sublimit_probe(model):
    """Small torques that stay off the joint stops, so inertia scaling is visible.
    Not a controller change and not the existing 70 N·m knee test.
    """
    def observe_spine(model, data):
        out = {}
        for joint in ("lumbar", "thoracic"):
            j = model.joint(joint)
            adr = int(j.qposadr[0])
            out[joint + "_angle_rad"] = ball_angle(data.qpos[adr : adr + 4])
        return out

    def observe_knee(model, data):
        j = model.joint("knee_l")
        adr = int(j.qposadr[0])
        return dict(knee_l_rad=float(data.qpos[adr]), sole_l_z=sole_min_z(model, data, "foot_l_geom"))

    spine = torque_test(
        model, 0.40,
        {"lumbar_x_motor": 2.0, "thoracic_x_motor": 2.0},
        with_harness=False,
        observe_fn=observe_spine,
    )
    knee = torque_test(
        model, 0.30,
        {"knee_l_motor": 8.0},
        with_harness=True,
        observe_fn=observe_knee,
    )
    return dict(spine=spine, knee=knee)


def provenance(base: mujoco.MjModel, assumptions: dict) -> str:
    before = mujoco.MjModel.from_xml_path(str(BEFORE))
    mass_delta = float(np.max(np.abs(base.body_mass - before.body_mass)))
    lines = []
    lines.append(
        f"PROVEN, in this repo: compiled `main` total mass is {float(np.sum(base.body_mass)):.12f} kg. "
        f"`reports/collision/model_before.xml` compiled total is {float(np.sum(before.body_mass)):.12f} kg. "
        f"Max per-body absolute mass difference is {mass_delta:.3e} kg. "
        "So the current explicit inertials are the pre-collision compiled masses, not a new mesh-density integration."
    )
    lines.append("")
    lines.append(
        "PROVEN, in this repo: `scripts/build_collision_surfaces.py` loads `reports/collision/model_before.xml`, "
        "writes an `<inertial>` per body from `model.body_mass`, `body_inertia`, `body_ipos`, `body_iquat`, "
        "and asserts the rebuilt model matches those arrays (`rtol=1e-10`). "
        "`docs/COLLISIONS.md` states that explicit inertias preserve the original mass, COM and inertia. "
        "The pre-change model has `inertiafromgeom=\"true\"` and capsule/sphere/box geoms whose densities are the values in `physics/assumptions.json`."
    )
    lines.append("")
    # density * MuJoCo primitive volume
    dens = assumptions["densities_kg_m3"]
    lines.append("PROVEN, compiled `model_before.xml` mass equals `density * MuJoCo geom volume` (not the intended mass):")
    lines.append("")
    lines.append("| body | intended | density * geom volume | compiled | volume kind |")
    lines.append("|---|---:|---:|---:|---|")
    for i in range(before.ngeom):
        name = mujoco.mj_id2name(before, mujoco.mjtObj.mjOBJ_GEOM, i)
        if name in (None, "floor"):
            continue
        body_id = int(before.geom_bodyid[i])
        body = mujoco.mj_id2name(before, mujoco.mjtObj.mjOBJ_BODY, body_id)
        gtype = int(before.geom_type[i])
        size = before.geom_size[i]
        if gtype == int(mujoco.mjtGeom.mjGEOM_CAPSULE):
            r, half = float(size[0]), float(size[1])
            vol = np.pi * r * r * (2 * half) + (4.0 / 3.0) * np.pi * r ** 3
            kind = "capsule (cylinder length = fromto, caps extra)"
        elif gtype == int(mujoco.mjtGeom.mjGEOM_SPHERE):
            r = float(size[0])
            vol = (4.0 / 3.0) * np.pi * r ** 3
            kind = "sphere"
        elif gtype == int(mujoco.mjtGeom.mjGEOM_BOX):
            vol = float(8.0 * size[0] * size[1] * size[2])
            kind = "box"
        else:
            continue
        m_vol = float(dens[body] * vol)
        m_c = float(before.body_mass[body_id])
        m_i = float(assumptions["masses_kg"][body])
        lines.append(f"| {body} | {m_i:.6f} | {m_vol:.6f} | {m_c:.6f} | {kind} |")
    lines.append("")
    lines.append(
        "PROVEN by executing the original generator, which is **not** in this git history "
        "(the repo imported the already-built MJCF in commit `75ef16b`): "
        "`/workspace/blender/body_lab/humanoid_alpha/scripts/02_build_mjcf.py` sets "
        "`density = intended_mass / capsule_volume(length, r)` with "
        "`h = max(length - 2r, 1e-6)` and `V = pi r^2 h + 4/3 pi r^3`. "
        "That treats `length` as the outer capsule length **including** both hemispherical caps. "
        "MuJoCo `fromto` capsules use `length` as the cylinder axis and add the caps outside it, so the geom is heavier whenever `length > 2r`. "
        "Pelvis uses a hard-coded length 0.12 m (shorter than the diameter, so the density volume collapses to a sphere) while the geom is a short capsule. "
        "`torso_lower` length 0.126 m is shorter than diameter 0.18 m, same sphere-vs-capsule gap (the largest torso error). "
        "Head density uses that capsule formula on `neck_head` length 0.2085 m, but the geom is a sphere, so the head is the only body **lighter** than intended. "
        "Feet use the capsule formula for density and a box geom. "
        "Recomputing every `assumptions.json` density from that function and `measurements.json` segment lengths matched bit-exactly (max relative error 0) on this machine."
    )
    lines.append("")
    lines.append(
        "HYPOTHESIS: none required for the 60→89 kg gap. The collision-surface rebuild did not create the extra mass; it preserved it. "
        "A non-uniform global scale of the 89 kg model would not recover `masses_kg`, because head must go up (ratio 0.808) while `torso_upper` must go down (ratio 2.197)."
    )
    return "\n".join(lines)


def main() -> int:
    assumptions = json.loads(ASSUMPTIONS.read_text())
    intended = assumptions["masses_kg"]
    base = load_main_baseline()
    exp = mujoco.MjModel.from_xml_path(str(MODEL))
    base_rows = body_rows(base)
    exp_rows = body_rows(exp)
    before = mass_table(base_rows, intended)
    after = []
    for r in exp_rows:
        m_i = float(intended[r["body"]])
        after.append(
            dict(
                body=r["body"],
                compiled_mass=r["mass"],
                intended_mass=m_i,
                abs_err=abs(r["mass"] - m_i),
                inertia_valid=triangle_ok(np.asarray(r["inertia"])),
                finite=bool(np.isfinite(r["mass"]) and np.all(np.isfinite(r["inertia"]))),
            )
        )
    invariants = check_invariants(base, exp)
    # mass-match checks
    mass_ok = all(r["abs_err"] <= MASS_ATOL and r["finite"] and r["inertia_valid"] and r["compiled_mass"] > 0 for r in after)
    invariants.append(
        dict(
            name="per_body_mass_matches_masses_kg",
            pass_=mass_ok,
            detail=f"atol={MASS_ATOL} max abs err={max(r['abs_err'] for r in after):.3e}",
        )
    )
    total = float(sum(r["compiled_mass"] for r in after))
    invariants.append(
        dict(
            name="total_mass_60",
            pass_=abs(total - 60.0) <= 1e-6,
            detail=f"compiled total {total:.12f}",
        )
    )
    print("running A/B ...", flush=True)
    behavior = dict(
        fall={"A": zero_control(base), "B": zero_control(exp)},
        harness={"A": harness_hold(base), "B": harness_hold(exp)},
        lumbar={"A": lumbar_thoracic(base), "B": lumbar_thoracic(exp)},
        knee={"A": knee_test(base), "B": knee_test(exp)},
        sublimit={"A": sublimit_probe(base), "B": sublimit_probe(exp)},
    )
    head = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip()
    branch = subprocess.check_output(["git", "rev-parse", "--abbrev-ref", "HEAD"], cwd=ROOT, text=True).strip()
    failures = []
    for c in invariants:
        if not c["pass_"]:
            failures.append(f"Invariant FAIL: {c['name']}: {c['detail']}")
    # unexpected: non-finite behavior samples
    def walk_samples(kind, label, result):
        if not isinstance(result, dict):
            return
        if "samples" in result:
            for s in result["samples"]:
                for k, v in s.items():
                    if isinstance(v, float) and not np.isfinite(v):
                        failures.append(f"non-finite {kind} {label} {k} at t={s.get('t')}")
        for k, v in result.items():
            if k == "samples":
                continue
            if isinstance(v, dict):
                walk_samples(kind + "." + k, label, v)

    for kind, pair in behavior.items():
        for label, result in pair.items():
            walk_samples(kind, label, result)
    ha, hb = behavior["harness"]["A"], behavior["harness"]["B"]
    if not hb["holds_validator_criterion"]:
        failures.append(
            "HARNESS on the 60 kg model does not meet the existing validator hold criterion "
            f"(min z={hb['min_z']:.4f}, max tilt={hb['max_tilt']:.4f}). Gains were not changed."
        )
    else:
        failures.append(
            "HARNESS still meets the existing 5 s hold criterion on the 60 kg model "
            f"(min z={hb['min_z']:.4f}, max tilt={hb['max_tilt']:.4f}) with unchanged gains. "
            f"End sag {hb['end_sag_m']:.4f} m vs baseline {ha['end_sag_m']:.4f} m. "
            "This is not a retune; the spring was already strong relative to either weight."
        )
    failures.append(
        "tests/test_collisions.py::test_original_dynamics_properties_and_actuators_preserved "
        "asserts compiled body_mass/body_inertia still match reports/collision/model_before.xml. "
        "That assertion is expected to FAIL after this mass correction. It was not edited. "
        "Measured failure: body_mass, 16/17 bodies differ, max abs diff 13.14080235 kg (torso_upper 24.121195 -> 10.980392)."
    )
    failures.append(
        "Existing tests were run and not retuned. `python -m unittest discover -s tests`: 26 tests, 25 passed, 1 failed (the collision mass-preservation test above). "
        "Human-control test passed. `scripts/validate_harness.py` all_pass true on the 60 kg model with unchanged gains: "
        "harness min pelvis z 0.905654, end z 0.907886, end tilt 0.035872 rad, max tilt 0.132457 rad, "
        "raw fall to z<0.55 at 0.502 s, knee 70 N·m lift still clears the left foot (sole_l 0.4957 m at 0.8 s) with right foot on the floor. "
        "Those validators rewrite reports/harness, reports/human_control/validation.json (xfrc_peak_abs 1097.646 -> 884.542 N) and reports/collision/controlled_motor_contact.json (contact_off_peak_m 0.094349 -> 0.102681). Those rewrites were restored and are not part of this branch; the numbers are recorded here."
    )
    payload = dict(
        branch=branch,
        head=head,
        method=(
            "Every body already had an explicit inertial element whose compiled mass was finite and positive "
            "and whose diaginertia (principal moments about the existing COM) was finite, positive, and satisfied "
            "the triangle inequalities, so the tensor is positive definite and physically realizable. "
            "No body needed a fallback. For each body independently, "
            "s = m_intended / m_compiled_baseline, new mass = m_intended (from physics/assumptions.json masses_kg), "
            "new diaginertia = s * old diaginertia. inertial pos and quat were not edited. "
            "Scaling a valid principal inertia by a positive scalar preserves the COM, the principal-axis directions, "
            "the radii of gyration, positive-definiteness, and the triangle inequalities. "
            "inertiafromgeom remains auto; the explicit inertial overrides geom density (the head sphere still carries "
            "its old density attribute, which does not add mass while the inertial element is present — compiled head mass matches masses_kg). "
            "Collision geoms were not edited."
        ),
        before_table=before,
        before_total=dict(
            current=float(sum(r["current_mass"] for r in before)),
            intended=float(sum(r["intended_mass"] for r in before)),
            ratio=float(sum(r["current_mass"] for r in before) / sum(r["intended_mass"] for r in before)),
        ),
        after_table=after,
        after_total=dict(
            compiled=total,
            intended=float(sum(r["intended_mass"] for r in after)),
            abs_err=abs(total - float(sum(r["intended_mass"] for r in after))),
        ),
        invariants=invariants,
        **behavior,
        provenance_md=provenance(base, assumptions),
        failures_md="\n".join(f"- {f}" for f in failures),
    )
    OUT.mkdir(parents=True, exist_ok=True)
    # json-friendly: pass_ key
    (OUT / "results.json").write_text(json.dumps(payload, indent=2) + "\n")
    report = write_report(payload)
    (OUT / "REPORT.md").write_text(report)
    print(report)
    failed = [c for c in invariants if not c["pass_"]]
    print("invariant failures", len(failed))
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
