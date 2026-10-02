"""Optional ASSISTED stand layer: posture PD + COM Jacobian balance above RAW u_cmd.

When disabled, the runtime must not call compute(); RAW act writes data.ctrl directly.
No qpos writes during simulation — only reset_to_stand() at enable / explicit reset.
"""
from __future__ import annotations

from typing import Any

import numpy as np
import mujoco


def _quat_conj(q: np.ndarray) -> np.ndarray:
    return np.array([q[0], -q[1], -q[2], -q[3]], dtype=np.float64)


def _quat_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array(
        [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ],
        dtype=np.float64,
    )


def _quat_err_vec(q_des: np.ndarray, q: np.ndarray) -> np.ndarray:
    """Axis-scaled quaternion error (≈ 2*xyz) mapping to ball joint angular DOFs."""
    e = _quat_mul(q_des, _quat_conj(q))
    if e[0] < 0:
        e = -e
    return 2.0 * e[1:]


def _class_of(name: str) -> str:
    for key in (
        "ankle",
        "knee",
        "hip",
        "lumbar",
        "thoracic",
        "neck",
        "shoulder",
        "elbow",
        "wrist",
    ):
        if key in name:
            return key
    return "wrist"


class AssistedStandController:
    """Hold near model.qpos0 with joint PD + subtree-COM Jacobian balance.

    Tuned gains (CEM + modest vertical COM, mj_resetData each trial): ≥10 s
    continuous upright (pelvis z ≥ 0.70 m, tilt ≤ 0.55 rad) on CURRENT model.
    Hold is finite (~10 s) — ankle budget / soft contacts still cap recovery.
    """

    # Posture PD (kp, kd) by actuator class — priority on legs/spine, low on arms/neck.
    DEFAULT_GAINS = {
        "ankle": (139.26658665680378, 139.26658665680378 * 0.04052040345609933),
        "knee": (294.4638411981989, 294.4638411981989 * 0.06932936198593045),
        "hip": (128.52840205649147, 128.52840205649147 * 0.0619194677482105),
        "lumbar": (38.14350552818898, 38.14350552818898 * 0.07949051587498708),
        "thoracic": (35.795525778161306, 35.795525778161306 * 0.08),
        "neck": (35.795525778161306 * 0.4, 35.795525778161306 * 0.04),
        "shoulder": (35.795525778161306 * 0.6, 35.795525778161306 * 0.06),
        "elbow": (35.795525778161306 * 0.5, 35.795525778161306 * 0.05),
        "wrist": (35.795525778161306 * 0.2, 35.795525778161306 * 0.02),
    }
    KP_COM = 4474.954047318542
    KD_COM = 133.76048972409114
    # Vertical COM via same subtree Jac (f_z); fights slow sink without qpos writes.
    KP_Z = 203.95393916096342
    KD_Z = 12.802615245609504
    TILT_FAIL = 1.0  # rad — begin fading assist
    Z_FAIL = 0.35  # m pelvis height
    NO_CONTACT_FAIL_STEPS = 80  # ~0.16 s at 500 Hz
    UPRIGHT_Z = 0.70
    UPRIGHT_TILT = 0.55

    def __init__(self, model: mujoco.MjModel):
        self.model = model
        self.enabled = False
        self._fade = 1.0
        self._no_contact = 0
        self._com_offset = np.zeros(2, dtype=np.float64)
        self._z_des = float(model.qpos0[2]) if model.nq > 2 else 0.9
        self._stand_qpos = model.qpos0.copy()
        self._jacp = np.zeros((3, model.nv), dtype=np.float64)
        self._last_diag: dict[str, Any] = {}
        self._build_maps(model)
        self._resolve_bodies(model)

    def _build_maps(self, model: mujoco.MjModel) -> None:
        self._hinge: list[tuple[int, int, int, float, float]] = []
        self._ball: dict[int, list[tuple[int, int, float, float, int, int]]] = {}
        self._balance_dofs: list[tuple[int, int]] = []  # (actuator_id, dof_index)
        balance_keys = ("ankle", "knee", "hip", "lumbar")
        for i in range(model.nu):
            name = model.actuator(i).name or ""
            jid = int(model.actuator_trnid[i, 0])
            jtype = int(model.jnt_type[jid])
            kp, kd = self.DEFAULT_GAINS[_class_of(name)]
            qadr = int(model.jnt_qposadr[jid])
            vadr = int(model.jnt_dofadr[jid])
            if jtype == 3:  # hinge
                self._hinge.append((i, qadr, vadr, kp, kd))
                if any(k in name for k in balance_keys):
                    self._balance_dofs.append((i, vadr))
            else:  # ball (and any multi-dof joint with gear axis)
                axis = int(np.argmax(np.abs(model.actuator_gear[i, :3])))
                self._ball.setdefault(jid, []).append((i, axis, kp, kd, qadr, vadr))
                if any(k in name for k in balance_keys):
                    self._balance_dofs.append((i, vadr + axis))
        self._ctrl_lo = model.actuator_ctrlrange[:, 0].copy()
        self._ctrl_hi = model.actuator_ctrlrange[:, 1].copy()

    def _resolve_bodies(self, model: mujoco.MjModel) -> None:
        self._pelvis = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "pelvis")
        self._foot_l = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "foot_l")
        self._foot_r = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "foot_r")
        self._floor = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "floor")
        fl = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "foot_l_geom")
        fr = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "foot_r_geom")
        self._foot_geoms = {int(fl), int(fr)}

    def _support_xy(self, data: mujoco.MjData) -> np.ndarray:
        """Support reference: midpoint of foot body origins (stable vs noisy CoP)."""
        return 0.5 * (data.xpos[self._foot_l, :2] + data.xpos[self._foot_r, :2])

    def _foot_contacts(self, data: mujoco.MjData) -> int:
        n = 0
        floor = self._floor
        feet = self._foot_geoms
        for i in range(data.ncon):
            c = data.contact[i]
            g = {int(c.geom1), int(c.geom2)}
            if floor in g and g & feet:
                n += 1
        return n

    def _pelvis_tilt(self, data: mujoco.MjData) -> float:
        w, x, y, z = data.xquat[self._pelvis]
        up_z = 1.0 - 2.0 * (x * x + y * y)
        return float(np.arccos(np.clip(up_z, -1.0, 1.0)))

    def _sole_lift_dz(self, model: mujoco.MjModel, data: mujoco.MjData) -> float:
        """Small root Δz so lowest foot mesh vertex sits at z=0 (optional seed fix)."""
        zmins = []
        for gname in ("foot_l_geom", "foot_r_geom"):
            gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, gname)
            mid = int(model.geom_dataid[gid])
            if mid < 0:
                continue
            adr = int(model.mesh_vertadr[mid])
            n = int(model.mesh_vertnum[mid])
            verts = model.mesh_vert[adr : adr + n]
            R = data.geom_xmat[gid].reshape(3, 3)
            pos = data.geom_xpos[gid]
            world = verts @ R.T + pos
            zmins.append(float(world[:, 2].min()))
        if not zmins:
            return 0.0
        return -min(zmins)

    def reset_to_stand(self, model: mujoco.MjModel, data: mujoco.MjData, sole_lift: bool = False) -> None:
        """Set qpos to stand seed (qpos0), zero velocity, mj_forward. ONLY at enable / explicit reset.

        sole_lift defaults False: CEM-tuned gains assume raw qpos0 (known ~1.6 mm sole penetration).
        """
        mujoco.mj_resetData(model, data)
        q = model.qpos0.copy()
        data.qpos[:] = q
        data.qvel[:] = 0
        data.ctrl[:] = 0
        mujoco.mj_forward(model, data)
        if sole_lift:
            dz = self._sole_lift_dz(model, data)
            if abs(dz) > 1e-6:
                data.qpos[2] += dz
                mujoco.mj_forward(model, data)
        self._stand_qpos = data.qpos.copy()
        self._com_offset = data.subtree_com[self._pelvis, :2] - self._support_xy(data)
        self._z_des = float(data.xpos[self._pelvis, 2])
        self._fade = 1.0
        self._no_contact = 0
        self._last_diag = {}

    def compute(self, model: mujoco.MjModel, data: mujoco.MjData, u_cmd: np.ndarray) -> np.ndarray:
        """Return clipped τ = u_cmd + fade * (τ_pd + τ_com). Does not write qpos."""
        u = np.asarray(u_cmd, dtype=np.float64).reshape(-1)
        if u.shape[0] != model.nu:
            raise ValueError(f"u_cmd length {u.shape[0]} != nu {model.nu}")

        tau_assist = np.zeros(model.nu, dtype=np.float64)
        q_des = self._stand_qpos

        # 1) Posture PD
        for aid, qa, va, kp, kd in self._hinge:
            tau_assist[aid] += kp * (q_des[qa] - data.qpos[qa]) - kd * data.qvel[va]
        for _jid, lst in self._ball.items():
            qa, va = lst[0][4], lst[0][5]
            err = _quat_err_vec(q_des[qa : qa + 4], data.qpos[qa : qa + 4])
            w = data.qvel[va : va + 3]
            for aid, axis, kp, kd, _, _ in lst:
                tau_assist[aid] += kp * err[axis] - kd * w[axis]

        # 2) COM balance via subtree Jacobian XY + modest vertical (ankles/knees/hips/lumbar)
        mujoco.mj_jacSubtreeCom(model, data, self._jacp, self._pelvis)
        support = self._support_xy(data)
        com_xy = data.subtree_com[self._pelvis, :2]
        # Desire rest offset relative to current support
        e_xy = (support + self._com_offset) - com_xy
        com_vel = self._jacp @ data.qvel
        # Root vertical velocity (free joint DOF 2); height error vs stand seed.
        vz = float(data.qvel[2])
        ez = float(self._z_des) - float(data.xpos[self._pelvis, 2])
        f = np.array(
            [
                self.KP_COM * e_xy[0] - self.KD_COM * com_vel[0],
                self.KP_COM * e_xy[1] - self.KD_COM * com_vel[1],
                self.KP_Z * ez - self.KD_Z * vz,
            ],
            dtype=np.float64,
        )
        tau_dof = self._jacp.T @ f
        for aid, dof in self._balance_dofs:
            tau_assist[aid] += tau_dof[dof]

        # 3) Fail-safe fade
        tilt = self._pelvis_tilt(data)
        pelvis_z = float(data.xpos[self._pelvis, 2])
        ncon_feet = self._foot_contacts(data)
        if ncon_feet == 0:
            self._no_contact += 1
        else:
            self._no_contact = 0
        if tilt > self.TILT_FAIL or pelvis_z < self.Z_FAIL or self._no_contact > self.NO_CONTACT_FAIL_STEPS:
            self._fade = max(0.0, self._fade - 0.05)
        else:
            self._fade = min(1.0, self._fade + 0.1)

        tau = u + self._fade * tau_assist
        tau = np.clip(tau, self._ctrl_lo, self._ctrl_hi)

        self._last_diag = dict(
            com_xy=com_xy.copy(),
            support_xy=support.copy(),
            e_xy=(-e_xy).copy(),  # COM − (support+offset), signed lean
            tilt=tilt,
            pelvis_z=pelvis_z,
            ncon_feet=ncon_feet,
            tau_norm=float(np.linalg.norm(tau)),
            fade=float(self._fade),
            enabled=bool(self.enabled),
        )
        return tau

    @property
    def diagnostics(self) -> dict[str, Any]:
        d = dict(self._last_diag)
        d["enabled"] = bool(self.enabled)
        return d
