"""Assisted-stand investigation helpers (experimental, not wired into the runtime).

Everything here reads the repository model; nothing writes to physics/model.
The flat-sole variant is built as an XML string in memory.
"""
from __future__ import annotations

from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
MODEL = ROOT / "physics" / "model" / "humanoid_alpha.xml"


def load_model(variant: str = "mesh") -> mujoco.MjModel:
    """'mesh' = repository model as is; 'flat_sole' = same model, floor touches a box sole instead of the foot hull."""
    if variant == "mesh":
        return mujoco.MjModel.from_xml_path(str(MODEL))
    if variant != "flat_sole":
        raise ValueError(variant)
    src = MODEL.read_text().replace('file="../../assets/collision/', f'file="{ROOT}/assets/collision/')
    for side, sx in (("l", 1), ("r", -1)):
        mesh = f'<geom name="foot_{side}_geom" type="mesh" mesh="foot_{side}_surface"'
        assert mesh in src
        src = src.replace(mesh, mesh + ' contype="0" conaffinity="0"')
        # Box over the measured flat sole band of the hull at qpos0 (world x 0.114..0.187 mirrored,
        # y -0.174..0.032, bottom z -0.0016); foot body frame is the world frame at rest.
        guard = f'<geom name="foot_{side}_joint_guard"'
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
    """Raise only the root so the static floor force equals body weight. Returns the lift in metres."""
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


def pitch_back_deg(d, body) -> float:
    """Sagittal lean of the body z-axis; + = backward (+Y; she faces -Y in MuJoCo)."""
    r = d.xmat[body].reshape(3, 3)
    return float(np.degrees(np.arctan2(r[1, 2], r[2, 2])))


class StanceID:
    """Floating-base inverse dynamics for double stance. Actuator torques only, clipped to ctrlrange.

    qacc_des: joint PD (acceleration units) + root task (COM xy over the feet, pelvis height, pelvis
    orientation). Stance leg accelerations come from the foot-fixed constraint. Contact forces:
      cop='corners': 4 sole-corner normal forces >= 0 per foot (CoP stays inside the sole),
                     free tangential force and yaw moment; angular root rows have weight w_ang.
      cop='free':    unconstrained 6D wrench per foot (min-norm).
    tau = M qacc_des + c - G lambda. No xfrc, no qpos/qvel writes.
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
            raise ValueError("cop='corners' needs the flat_sole variant")
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
        lam = None
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
