"""Experimental stand v2: flat-sole contact variant + floating-base inverse-dynamics stance.

Used only by the stand-v2 / debug-v2 launcher modes (``control_console.py --stand-v2``).
The repository MJCF is not modified: the flat-sole variant is built as an XML string.
Investigation and numbers: reports/ASSISTED_STAND_CLAUDE.md.

Actuator torques only (clipped to ctrlrange). No xfrc_applied, no qpos/qvel writes except the
stand seed at enable/reset, same contract as assisted_stand.AssistedStandController.
"""
from __future__ import annotations

from pathlib import Path
from typing import Any

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
MODEL = ROOT / "physics" / "model" / "humanoid_alpha.xml"


def load_flat_sole_model(path: Path = MODEL) -> mujoco.MjModel:
    """Same model, but the floor touches one box per foot instead of the convex foot hull.

    The hull gives MuJoCo 1-3 floor contacts per foot (no usable centre of pressure). The box
    covers the measured flat band of the hull at qpos0: world x 0.114..0.187 (mirrored),
    y -0.174..0.032, bottom z -0.0016; the foot body frame equals the world frame at rest.
    The hull keeps its explicit joint-guard pair and stays visible; mass is unchanged.
    """
    src = path.read_text().replace('file="../../assets/collision/', f'file="{ROOT}/assets/collision/')
    for side, sx in (("l", 1), ("r", -1)):
        mesh = f'<geom name="foot_{side}_geom" type="mesh" mesh="foot_{side}_surface"'
        guard = f'<geom name="foot_{side}_joint_guard"'
        if mesh not in src or guard not in src:
            raise RuntimeError(f"foot_{side} geoms not found in {path}")
        src = src.replace(mesh, mesh + ' contype="0" conaffinity="0"')
        src = src.replace(guard, f'<geom name="sole_{side}" type="box" pos="{sx * 0.0275:.4f} -0.0725 -0.0661" '
                                 f'size="0.036 0.103 0.010" friction="0.8 0.1 0.1" rgba="0.2 0.2 0.2 1"/>\n            ' + guard)
    return mujoco.MjModel.from_xml_string(src)


def floor_fz(m, d) -> float:
    fz = 0.0
    f6 = np.zeros(6)
    for i in range(d.ncon):
        mujoco.mj_contactForce(m, d, i, f6)
        fz += (d.contact[i].frame.reshape(3, 3).T @ f6[:3])[2]
    return fz


def equilibrium_seed(m, d, q0=None) -> float:
    """Raise only the root so the static floor force equals body weight. Returns the lift in metres.

    Raw qpos0 starts with the soles inside the contact margin: the first step pushes with ~25x
    body weight and throws the body.
    """
    q0 = m.qpos0 if q0 is None else q0
    weight = float(m.body_subtreemass[1]) * abs(m.opt.gravity[2])
    lo, hi = 0.0, 0.02
    for _ in range(40):
        mid = 0.5 * (lo + hi)
        d.qpos[:] = q0
        d.qpos[2] += mid
        d.qvel[:] = 0
        mujoco.mj_forward(m, d)
        lo, hi = (mid, hi) if floor_fz(m, d) > weight else (lo, mid)
    d.qpos[:] = q0
    d.qpos[2] += 0.5 * (lo + hi)
    d.qvel[:] = 0
    d.ctrl[:] = 0
    mujoco.mj_forward(m, d)
    return 0.5 * (lo + hi)


def actuator_dofs(m) -> np.ndarray:
    out = []
    for i in range(m.nu):
        j = m.actuator_trnid[i, 0]
        axis = int(np.argmax(np.abs(m.actuator_gear[i, :3]))) if m.jnt_type[j] == mujoco.mjtJoint.mjJNT_BALL else 0
        out.append(m.jnt_dofadr[j] + axis)
    return np.array(out)


def tilt_deg(d, body) -> float:
    return float(np.degrees(np.arccos(np.clip(d.xmat[body].reshape(3, 3)[2, 2], -1, 1))))


class StanceID:
    """Floating-base inverse dynamics for double stance.

    qacc_des: joint PD (acceleration units) + root task (COM xy over the feet, pelvis height, pelvis
    orientation). Stance leg accelerations come from the foot-fixed constraint. Contact forces:
      cop='corners': 4 sole-corner normal forces >= 0 per foot (CoP stays inside the sole),
                     free tangential force and yaw moment; angular root rows have weight w_ang.
      cop='free':    unconstrained 6D wrench per foot (min-norm).
    tau = M qacc_des + c - G lambda, clipped to ctrlrange.
    """

    def __init__(self, m, cop="corners", kp_j=100.0, kd_j=20.0, kp_com=30.0, kd_com=11.0, kp_z=60.0,
                 kd_z=15.0, kp_o=60.0, kd_o=16.0, w_ang=0.3, mom_w=20.0, dls=1e-3, lift_h=0.02):
        self.m, self.cop = m, cop
        self.kp_j, self.kd_j, self.kp_com, self.kd_com = kp_j, kd_j, kp_com, kd_com
        self.kp_z, self.kd_z, self.kp_o, self.kd_o = kp_z, kd_z, kp_o, kd_o
        self.w_ang, self.mom_w, self.dls, self.lift_h = w_ang, mom_w, dls, lift_h
        self.P = m.body("pelvis").id
        self.feet = [m.body("foot_l").id, m.body("foot_r").id]
        has_sole = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "sole_l") >= 0
        if cop == "corners" and not has_sole:
            raise ValueError("cop='corners' needs the flat-sole model")
        self.sole = [m.geom("sole_l").id, m.geom("sole_r").id] if has_sole else None
        self.M = np.zeros((m.nv, m.nv))
        self.jp = np.zeros((3, m.nv))
        self.jr = np.zeros((3, m.nv))
        self.adof = actuator_dofs(m)
        self.lo, self.hi = m.actuator_ctrlrange[:, 0], m.actuator_ctrlrange[:, 1]
        self.joints = [j for j in range(m.njnt) if m.jnt_type[j] != mujoco.mjtJoint.mjJNT_FREE]
        self.leg = [self._leg_dofs("l"), self._leg_dofs("r")]
        self.diag = {}

    def _leg_dofs(self, side):
        out = []
        for n in (f"hip_{side}", f"knee_{side}", f"ankle_{side}_dp", f"ankle_{side}_ie"):
            j = self.m.joint(n).id
            k = 3 if self.m.jnt_type[j] == mujoco.mjtJoint.mjJNT_BALL else 1
            out += range(self.m.jnt_dofadr[j], self.m.jnt_dofadr[j] + k)
        return np.array(out)

    def reset(self, d):
        self.q_des = d.qpos.copy()
        self.z_des = float(d.qpos[2])
        self.com_off = d.subtree_com[self.P, :2] - self._mid(d)
        self.foot_z0 = [float(d.xpos[f, 2]) for f in self.feet]

    def _mid(self, d):
        return 0.5 * (d.xpos[self.feet[0], :2] + d.xpos[self.feet[1], :2])

    def compute(self, m, d):
        nv = m.nv
        qacc = np.zeros(nv)
        e = np.zeros(3)
        for j in self.joints:
            qa, va = m.jnt_qposadr[j], m.jnt_dofadr[j]
            if m.jnt_type[j] == mujoco.mjtJoint.mjJNT_BALL:
                mujoco.mju_subQuat(e, self.q_des[qa:qa + 4], d.qpos[qa:qa + 4])
                qacc[va:va + 3] = self.kp_j * e - self.kd_j * d.qvel[va:va + 3]
            else:
                qacc[va] = self.kp_j * (self.q_des[qa] - d.qpos[qa]) - self.kd_j * d.qvel[va]
        mujoco.mj_jacSubtreeCom(m, d, self.jp, self.P)
        comv = self.jp @ d.qvel
        com = d.subtree_com[self.P]
        qacc[0:2] = self.kp_com * (self._mid(d) + self.com_off - com[:2]) - self.kd_com * comv[:2]
        qacc[2] = self.kp_z * (self.z_des - d.qpos[2]) - self.kd_z * d.qvel[2]
        mujoco.mju_subQuat(e, self.q_des[3:7], d.qpos[3:7])
        qacc[3:6] = self.kp_o * e - self.kd_o * d.qvel[3:6]

        stance = [(d.xpos[f, 2] - z0) < self.lift_h for f, z0 in zip(self.feet, self.foot_z0)]
        cols, normal = [], []
        for k, f in enumerate(self.feet):
            if not stance[k]:
                continue
            pt = d.xpos[f].copy()
            pt[2] = 0.0
            mujoco.mj_jac(m, d, self.jp, self.jr, pt, f)
            jf = np.vstack([self.jp, self.jr])
            legs = self.leg[k]
            other = np.setdiff1d(np.arange(nv), legs)
            rhs = -jf[:, other] @ qacc[other] - 20.0 * (jf @ d.qvel)  # stance foot stays put
            a = jf[:, legs]
            qacc[legs] = np.linalg.solve(a.T @ a + self.dls * np.eye(len(legs)), a.T @ rhs)
            if self.cop == "corners":
                g = self.sole[k]
                rot, ctr = d.geom_xmat[g].reshape(3, 3), d.geom_xpos[g]
                sx, sy, sz = m.geom_size[g]
                for p in [ctr + rot @ np.array([u * sx, v * sy, -sz]) for u in (-1, 1) for v in (-1, 1)]:
                    mujoco.mj_jac(m, d, self.jp, self.jr, p, f)
                    cols.append(self.jp.T @ np.array([0, 0, 1.0]))
                    normal.append(True)
                mujoco.mj_jac(m, d, self.jp, self.jr, ctr + rot @ np.array([0, 0, -sz]), f)
                for v in (np.array([1.0, 0, 0]), np.array([0, 1.0, 0])):
                    cols.append(self.jp.T @ v)
                    normal.append(False)
                cols.append(self.jr.T @ np.array([0, 0, 1.0]))
                normal.append(False)
            else:
                for r in range(6):
                    cols.append(jf[r])
                    normal.append(False)
        mujoco.mj_fullM(m, d, self.M)
        f_all = self.M @ qacc + d.qfrc_bias
        if cols:
            g = np.array(cols).T
            normal = np.array(normal)
            if self.cop == "corners":
                wr = np.r_[np.ones(3), np.full(3, self.w_ang)]
                a, b = wr[:, None] * g[:6], wr * f_all[:6]
                free = np.ones(g.shape[1], bool)
                for _ in range(10):  # active set for normal forces >= 0
                    af = a[:, free]
                    lam = np.zeros(g.shape[1])
                    lam[free] = np.linalg.solve(af.T @ af + 1e-6 * np.eye(af.shape[1]), af.T @ b)
                    neg = free & normal & (lam < 0)
                    if not neg.any():
                        break
                    free &= ~neg
            else:
                a = g[:6]
                w = np.tile(np.r_[np.ones(3), np.full(3, self.mom_w)], len(cols) // 6)
                wi = np.diag(1 / w)
                lam = wi @ a.T @ np.linalg.solve(a @ wi @ a.T + 1e-4 * np.eye(6), f_all[:6])
            f_all = f_all - g @ lam
        tau = np.clip(f_all[self.adof], self.lo, self.hi)
        self.diag = dict(stance=int(sum(stance)), saturated=int(np.sum((tau <= self.lo + 1e-9) | (tau >= self.hi - 1e-9))))
        return tau


class StanceController(StanceID):
    """StanceID used by the launcher modes, with three fixes found under disturbances:

    1. u_cmd on non-leg joints is a known input: it accelerates those joints and the whole-body
       solve compensates its reaction (head, arms, torso moved by the user).
    2. The COM task is solved jointly with root translation and the stance legs (pelvis != COM
       when the upper body moves); stance feet stay fixed by the same solve.
    3. Torque-feasible task scaling: if the task needs more than ctrlrange, all task accelerations
       are scaled by one alpha in [0, 1] so torques fit, instead of clipping into an inconsistent
       command (clipping made the legs kick and the solve blow up). Only sole corners actually on
       the floor carry force; a fast-moving or lifted foot is not a stance foot.
    """

    CORNER_DZ = 0.004  # m
    FOOT_SPEED_MAX = 0.3  # m/s

    def __init__(self, m, **kw):
        super().__init__(m, cop="corners", **kw)
        self.Jcom = np.zeros((3, m.nv))
        self.upper = np.setdiff1d(np.arange(6, m.nv), np.r_[self.leg[0], self.leg[1]])

    def _corners(self, d, k):
        g = self.sole[k]
        rot, ctr = d.geom_xmat[g].reshape(3, 3), d.geom_xpos[g]
        sx, sy, sz = self.m.geom_size[g]
        return [ctr + rot @ np.array([a * sx, b * sy, -sz]) for a in (-1, 1) for b in (-1, 1)], ctr + rot @ np.array([0, 0, -sz])

    def _task_qacc(self, m, d, ugen):
        nv = m.nv
        qacc = np.zeros(nv)
        e = np.zeros(3)
        for j in self.joints:
            qa, va = m.jnt_qposadr[j], m.jnt_dofadr[j]
            if m.jnt_type[j] == mujoco.mjtJoint.mjJNT_BALL:
                mujoco.mju_subQuat(e, self.q_des[qa:qa + 4], d.qpos[qa:qa + 4])
                qacc[va:va + 3] = self.kp_j * e - self.kd_j * d.qvel[va:va + 3]
            else:
                qacc[va] = self.kp_j * (self.q_des[qa] - d.qpos[qa]) - self.kd_j * d.qvel[va]
        up = self.upper
        qacc[up] += np.linalg.solve(self.M[np.ix_(up, up)], ugen[up])
        mujoco.mju_subQuat(e, self.q_des[3:7], d.qpos[3:7])
        qacc[3:6] = self.kp_o * e - self.kd_o * d.qvel[3:6]
        mujoco.mj_jacSubtreeCom(m, d, self.Jcom, self.P)
        comv = self.Jcom @ d.qvel
        com = d.subtree_com[self.P]
        a_com = np.r_[self.kp_com * (self._mid(d) + self.com_off - com[:2]) - self.kd_com * comv[:2],
                      self.kp_z * (self.z_des - d.qpos[2]) - self.kd_z * d.qvel[2]]
        stance, jf = [], {}
        for k, f in enumerate(self.feet):
            pt = d.xpos[f].copy()
            pt[2] = 0.0
            mujoco.mj_jac(m, d, self.jp, self.jr, pt, f)
            jf[k] = np.vstack([self.jp, self.jr])
            low = min(p[2] for p in self._corners(d, k)[0])
            stance.append(low < self.CORNER_DZ and np.linalg.norm(self.jp @ d.qvel) < self.FOOT_SPEED_MAX)
        unk = np.concatenate([np.arange(3)] + [self.leg[k] for k in range(2) if stance[k]])
        known = np.setdiff1d(np.arange(nv), unk)
        rows_a, rows_b = [self.Jcom[:, unk]], [a_com - self.Jcom[:, known] @ qacc[known]]
        for k in range(2):
            if stance[k]:
                rows_a.append(jf[k][:, unk])
                rows_b.append(-jf[k][:, known] @ qacc[known] - 20.0 * (jf[k] @ d.qvel))
        a, b = np.vstack(rows_a), np.concatenate(rows_b)
        w = np.r_[np.ones(3), np.full(a.shape[0] - 3, 10.0)]  # feet-fixed rows dominate the COM task
        aw = w[:, None] * a
        qacc[unk] = np.linalg.solve(aw.T @ aw + self.dls * np.eye(len(unk)), aw.T @ (w * b))
        return qacc, stance

    def _distribute(self, m, d, f_all, stance):
        cols, normal = [], []
        for k, f in enumerate(self.feet):
            if not stance[k]:
                continue
            corners, ctr = self._corners(d, k)
            for p in corners:
                if p[2] > self.CORNER_DZ:
                    continue
                mujoco.mj_jac(m, d, self.jp, self.jr, p, f)
                cols.append(self.jp.T @ np.array([0, 0, 1.0]))
                normal.append(True)
            mujoco.mj_jac(m, d, self.jp, self.jr, ctr, f)
            for v in (np.array([1.0, 0, 0]), np.array([0, 1.0, 0])):
                cols.append(self.jp.T @ v)
                normal.append(False)
            cols.append(self.jr.T @ np.array([0, 0, 1.0]))
            normal.append(False)
        if not cols:
            return f_all
        g = np.array(cols).T
        normal = np.array(normal)
        wr = np.r_[np.ones(3), np.full(3, self.w_ang)]
        a, b = wr[:, None] * g[:6], wr * f_all[:6]
        free = np.ones(g.shape[1], bool)
        for _ in range(12):  # active set for normal forces >= 0
            af = a[:, free]
            lam = np.zeros(g.shape[1])
            lam[free] = np.linalg.solve(af.T @ af + 1e-6 * np.eye(af.shape[1]), af.T @ b)
            neg = free & normal & (lam < 0)
            if not neg.any():
                break
            free &= ~neg
        return f_all - g @ lam

    def compute(self, m, d, u=None):
        """Controller torque only; the caller adds u. tau_total = u + this, within ctrlrange when alpha < 1."""
        mujoco.mj_fullM(m, d, self.M)
        ugen = np.zeros(m.nv)
        if u is not None:
            np.add.at(ugen, self.adof, u)
        qacc, stance = self._task_qacc(m, d, ugen)
        base = d.qfrc_bias - ugen
        t1 = self._distribute(m, d, self.M @ qacc + base, stance)[self.adof]
        t0 = self._distribute(m, d, base.copy(), stance)[self.adof]  # hold against gravity, no task
        alpha, dt = 1.0, t1 - t0
        with np.errstate(divide="ignore", invalid="ignore"):
            for lim in (self.hi, self.lo):
                r = (lim - t0) / dt
                ok = np.isfinite(r) & (r > 0) & (np.sign(dt) == np.sign(lim - t0))
                if ok.any():
                    alpha = min(alpha, float(r[ok].min()))
        alpha = max(0.0, alpha)
        tau = t1 if alpha >= 1.0 else self._distribute(m, d, alpha * (self.M @ qacc) + base, stance)[self.adof]
        tau = np.clip(tau, self.lo, self.hi)
        self.diag = dict(stance=int(sum(stance)), alpha=round(alpha, 3),
                         saturated=int(np.sum((tau <= self.lo + 1e-9) | (tau >= self.hi - 1e-9))))
        return tau


class StandV2Assist:
    """Drop-in for AssistedStandController in the console (reset_to_stand / compute / diagnostics).

    tau = clip(u_cmd + fade * tau_stance). Fades to zero (limp, physics only) once the torso
    tilts past TORSO_FAIL or the pelvis drops below Z_FAIL; humanoid-assist reset re-seeds.
    """

    TORSO_FAIL_DEG = 45.0
    Z_FAIL = 0.55

    def __init__(self, model: mujoco.MjModel):
        self.model = model
        self.enabled = False
        self.stance = StanceController(model)
        self._pelvis = model.body("pelvis").id
        self._torso = model.body("torso_upper").id
        self._lo, self._hi = model.actuator_ctrlrange[:, 0], model.actuator_ctrlrange[:, 1]
        self._fade = 1.0
        self._seed_lift = 0.0
        self._last_diag: dict[str, Any] = {}

    def reset_to_stand(self, model, data, sole_lift=False):
        mujoco.mj_resetData(model, data)
        self._seed_lift = equilibrium_seed(model, data)
        self.stance.reset(data)
        self._fade = 1.0
        self._last_diag = {}

    def compute(self, model, data, u_cmd):
        u = np.asarray(u_cmd, dtype=np.float64).reshape(-1)
        if u.shape[0] != model.nu:
            raise ValueError(f"u_cmd length {u.shape[0]} != nu {model.nu}")
        tau_stance = self.stance.compute(model, data, u if self._fade > 0 else None)
        torso = tilt_deg(data, self._torso)
        pelvis_z = float(data.xpos[self._pelvis, 2])
        fallen = torso > self.TORSO_FAIL_DEG or pelvis_z < self.Z_FAIL
        self._fade = max(0.0, self._fade - 0.05) if fallen else min(1.0, self._fade + 0.1)
        tau = np.clip(u + self._fade * tau_stance, self._lo, self._hi)
        self._last_diag = dict(controller="stand_v2", pelvis_z=pelvis_z, pelvis_tilt_deg=tilt_deg(data, self._pelvis),
                               torso_tilt_deg=torso, stance_feet=self.stance.diag["stance"],
                               saturated=self.stance.diag["saturated"], task_scale=self.stance.diag["alpha"], fade=float(self._fade),
                               seed_lift_mm=round(self._seed_lift * 1000, 2))
        return tau

    @property
    def diagnostics(self) -> dict[str, Any]:
        d = dict(self._last_diag)
        d["enabled"] = bool(self.enabled)
        return d
