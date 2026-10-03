"""Virtual overhead harness: world-frame wrench on the pelvis, not a pose weld.

This is a physical support for motor exploration. It is not a balance solver
and it is not a walking controller. It never writes qpos or qvel.

MuJoCo ``data.xfrc_applied[body]`` is a 6-vector applied at that body's center
of mass, in the **world frame**. On MuJoCo 3.14 the layout is **force then
torque** ``[fx, fy, fz, tx, ty, tz]`` (checked against ``mj_applyFT``). The
wrench is rewritten every tick and cleared when the harness is disabled.

Actuator commands are not touched. Gravity is not touched. Contacts stay on.
There is no weld, no root lock, and no XY position spring.
"""
from __future__ import annotations

from typing import Any

import numpy as np
import mujoco


# Vertical spring-damper toward the stand pelvis height (root qpos z / qpos0).
# Weight of this model is about 874 N, so kp_z = 30000 N/m sags ~3 cm at rest.
# The force cap is above body weight so support can actually hold, and finite
# so the harness cannot become a rigid kinematic constraint.
KP_Z = 30000.0  # N/m
KD_Z = 1600.0  # N / (m/s)
MAX_FORCE = 2200.0  # N, |Fz| cap

# Horizontal: damping only. No position gain on x or y.
KD_XY = 25.0  # N / (m/s)
MAX_HORIZONTAL_FORCE = 400.0  # N, cap on the damping force magnitude

# Roll/pitch restoring torque from the body +Z axis toward world +Z.
# Magnitude of (up × ez) is sin(tilt), so KP_RP is N·m per unit sin(tilt).
# Yaw spring is zero. A small yaw damper keeps the free yaw from winding up
# without holding a heading.
KP_RP = 1800.0  # N·m
KD_RP = 90.0  # N·m / (rad/s) on world wx, wy
KD_YAW = 6.0  # N·m / (rad/s) on world wz
MAX_TORQUE = 350.0  # N·m, cap on the torque vector


def _body_up_world(quat_wxyz: np.ndarray) -> np.ndarray:
    """Body +Z axis expressed in the world frame. ``quat`` is (w, x, y, z)."""
    w, x, y, z = (float(v) for v in quat_wxyz)
    return np.array(
        [
            2.0 * (x * z + y * w),
            2.0 * (y * z - x * w),
            1.0 - 2.0 * (x * x + y * y),
        ],
        dtype=np.float64,
    )


class HarnessSupport:
    """External pelvis support. ``apply`` writes only ``xfrc_applied[pelvis]``."""

    def __init__(self, model: mujoco.MjModel):
        self.model = model
        self.enabled = False
        self.pelvis_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "pelvis"))
        if self.pelvis_id < 0:
            raise RuntimeError("body 'pelvis' not found")
        root = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "root"))
        if root < 0:
            raise RuntimeError("free joint 'root' not found")
        if int(model.jnt_bodyid[root]) != self.pelvis_id:
            raise RuntimeError("root joint is not on the pelvis; harness addresses would be wrong")
        if int(model.jnt_type[root]) != int(mujoco.mjtJoint.mjJNT_FREE):
            raise RuntimeError("root joint is not free")
        quat = np.asarray(model.body_quat[self.pelvis_id], dtype=np.float64)
        if not np.allclose(quat, [1.0, 0.0, 0.0, 0.0]):
            raise RuntimeError("pelvis body_quat is not identity; qpos quat would not be the body frame")
        self._qadr = int(model.jnt_qposadr[root])
        self._vadr = int(model.jnt_dofadr[root])
        self.z_des = float(model.qpos0[self._qadr + 2])
        self._diag: dict[str, Any] = self._empty_diag()

    @staticmethod
    def _empty_diag() -> dict[str, Any]:
        return dict(
            enabled=False,
            force=[0.0, 0.0, 0.0],
            torque=[0.0, 0.0, 0.0],
            pelvis_z=None,
            tilt=None,
            pelvis_xy=[None, None],
        )

    def seed_stand(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        """Set the stand pose once, at enable. Not called from ``apply``.

        Writes qpos (qpos0) and zeroes qvel/ctrl, then ``mj_forward``. The
        per-tick harness never does this.
        """
        mujoco.mj_resetData(model, data)
        data.qpos[:] = model.qpos0
        data.qvel[:] = 0
        data.ctrl[:] = 0
        data.xfrc_applied[:] = 0
        mujoco.mj_forward(model, data)
        self.z_des = float(data.qpos[self._qadr + 2])
        self._diag = self._empty_diag()
        self._diag["pelvis_z"] = self.z_des

    def enable(self) -> None:
        self.enabled = True
        self._diag["enabled"] = True

    def disable(self, data: mujoco.MjData) -> None:
        """Turn support off and clear the pelvis wrench so nothing stale remains."""
        self.enabled = False
        data.xfrc_applied[self.pelvis_id, :] = 0
        self._diag["enabled"] = False
        self._diag["force"] = [0.0, 0.0, 0.0]
        self._diag["torque"] = [0.0, 0.0, 0.0]

    def apply(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        """Write this tick's pelvis wrench. Does not modify qpos, qvel, ctrl, or gravity.

        Call before ``mj_step``. If the harness is disabled, the pelvis wrench
        is set to zero (no stale force) and nothing else is written.
        """
        if not self.enabled:
            data.xfrc_applied[self.pelvis_id, :] = 0
            self._diag["enabled"] = False
            self._diag["force"] = [0.0, 0.0, 0.0]
            self._diag["torque"] = [0.0, 0.0, 0.0]
            return

        qadr, vadr = self._qadr, self._vadr
        # Root free joint: qpos is [x, y, z, qw, qx, qy, qz], qvel is world
        # [vx, vy, vz, wx, wy, wz]. Pelvis body_quat is identity, so this is
        # the pelvis origin pose and the world spatial velocity of that joint.
        pos = data.qpos[qadr : qadr + 3]
        quat = data.qpos[qadr + 3 : qadr + 7]
        vel = data.qvel[vadr : vadr + 3]
        ang = data.qvel[vadr + 3 : vadr + 6]
        z = float(pos[2])
        up = _body_up_world(quat)
        tilt = float(np.arccos(np.clip(up[2], -1.0, 1.0)))

        # Fz: spring-damper to the stand height, world +Z. No feedforward fudge
        # beyond the spring; the cap stops it short of a kinematic lift.
        fz = KP_Z * (self.z_des - z) - KD_Z * float(vel[2])
        fz = float(np.clip(fz, -MAX_FORCE, MAX_FORCE))

        # Fxy: viscous damping only. A position term would pin the harness point.
        fxy = -KD_XY * np.asarray(vel[:2], dtype=np.float64)
        fxy_n = float(np.linalg.norm(fxy))
        if fxy_n > MAX_HORIZONTAL_FORCE > 0.0:
            fxy *= MAX_HORIZONTAL_FORCE / fxy_n

        # Torque, world frame. up × ez = (uy, -ux, 0): roll/pitch toward upright,
        # zero yaw. Angular damping then adds a weak yaw term.
        ez = np.array([0.0, 0.0, 1.0], dtype=np.float64)
        torque = KP_RP * np.cross(up, ez)
        torque[0] -= KD_RP * float(ang[0])
        torque[1] -= KD_RP * float(ang[1])
        torque[2] -= KD_YAW * float(ang[2])
        t_n = float(np.linalg.norm(torque))
        if t_n > MAX_TORQUE > 0.0:
            torque *= MAX_TORQUE / t_n

        # Layout verified on this MuJoCo: force (3) then torque (3), world frame, at body CoM.
        data.xfrc_applied[self.pelvis_id, 0] = float(fxy[0])
        data.xfrc_applied[self.pelvis_id, 1] = float(fxy[1])
        data.xfrc_applied[self.pelvis_id, 2] = fz
        data.xfrc_applied[self.pelvis_id, 3] = float(torque[0])
        data.xfrc_applied[self.pelvis_id, 4] = float(torque[1])
        data.xfrc_applied[self.pelvis_id, 5] = float(torque[2])

        self._diag = dict(
            enabled=True,
            force=[float(fxy[0]), float(fxy[1]), fz],
            torque=[float(torque[0]), float(torque[1]), float(torque[2])],
            pelvis_z=z,
            tilt=tilt,
            pelvis_xy=[float(pos[0]), float(pos[1])],
        )

    @property
    def diagnostics(self) -> dict[str, Any]:
        out = dict(self._diag)
        out["enabled"] = bool(self.enabled)
        out["pelvis_id"] = self.pelvis_id
        out["pelvis_name"] = "pelvis"
        out["z_des"] = float(self.z_des)
        return out
