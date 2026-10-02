"""Participant interface: reset / observe / apply_torques / step / render_observation.

External agents MUST NOT receive bpy, raw mjData mutation, Python exec,
or arbitrary full-state poke beyond the allowed initial_state schema.
"""
from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Any, Optional

import numpy as np

try:
    import mujoco
except ImportError as e:
    raise ImportError("mujoco required in venv") from e

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MODEL = ROOT / "model" / "humanoid_alpha.xml"
META_PATH = ROOT / "model_meta.json"
ASSUMPTIONS_PATH = ROOT / "assumptions.json"

MAX_APPLY_DURATION = 1.0  # seconds per apply_torques call
MIN_APPLY_DURATION = 0.0

_meta = json.loads(META_PATH.read_text()) if META_PATH.exists() else {}
ACTION_NAMES: list[str] = list(_meta.get("actuator_order") or _meta.get("actuator_names") or [])
ACTION_DIM = len(ACTION_NAMES)

_assumptions = json.loads(ASSUMPTIONS_PATH.read_text()) if ASSUMPTIONS_PATH.exists() else {}
_tlim = _assumptions.get("torque_limits_Nm", {})

# Per-actuator limits from model; fallback from assumptions classes
def _build_torque_limits(model: "mujoco.MjModel") -> np.ndarray:
    lim = np.zeros((model.nu, 2), dtype=np.float64)
    for i in range(model.nu):
        lim[i, 0] = float(model.actuator_ctrlrange[i, 0])
        lim[i, 1] = float(model.actuator_ctrlrange[i, 1])
    return lim


TORQUE_LIMITS: Optional[np.ndarray] = None  # filled on first env init


# Allowed initial_state keys (participant schema)
ALLOWED_INITIAL_KEYS = {
    "root_pos",          # (3,) world position override for freejoint
    "root_quat_wxyz",    # (4,)
    "root_lin_vel",      # (3,)
    "root_ang_vel",      # (3,)
    "joint_angles",      # dict name->float for hinges OR name->[w,x,y,z] for balls
    "hinge_qpos",        # dict joint_name -> float
}


class HumanoidEnv:
    """Torque-controlled humanoid. MuJoCo is sole physics SoT."""

    def __init__(self, model_path: str | Path | None = None, seed: int = 0):
        path = Path(model_path) if model_path else DEFAULT_MODEL
        self.model = mujoco.MjModel.from_xml_path(str(path))
        self.data = mujoco.MjData(self.model)
        self._rng = np.random.default_rng(seed)
        self._seed = int(seed)
        global TORQUE_LIMITS, ACTION_NAMES, ACTION_DIM
        TORQUE_LIMITS = _build_torque_limits(self.model)
        ACTION_NAMES = [
            mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
            for i in range(self.model.nu)
        ]
        ACTION_DIM = self.model.nu
        self._torque_limits = TORQUE_LIMITS.copy()
        self._joint_qpos_adr = {}
        self._joint_qvel_adr = {}
        self._joint_type = {}
        for i in range(self.model.njnt):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i)
            self._joint_qpos_adr[name] = int(self.model.jnt_qposadr[i])
            self._joint_qvel_adr[name] = int(self.model.jnt_dofadr[i])
            self._joint_type[name] = int(self.model.jnt_type[i])
        self._last_image_path: Optional[str] = None
        self._episode_actions: list = []
        mujoco.mj_forward(self.model, self.data)

    # --- Participant API ---

    def reset(
        self,
        seed: Optional[int] = None,
        initial_state: Optional[dict] = None,
    ) -> dict:
        """Reset simulation. initial_state limited to ALLOWED_INITIAL_KEYS schema."""
        if seed is not None:
            self._seed = int(seed)
            self._rng = np.random.default_rng(self._seed)
        mujoco.mj_resetData(self.model, self.data)
        self._episode_actions = []
        self._last_image_path = None

        if initial_state:
            unknown = set(initial_state.keys()) - ALLOWED_INITIAL_KEYS
            if unknown:
                raise ValueError(
                    f"initial_state keys not allowed: {sorted(unknown)}. "
                    f"Allowed: {sorted(ALLOWED_INITIAL_KEYS)}"
                )
            self._apply_initial_state(initial_state)

        mujoco.mj_forward(self.model, self.data)
        return self.observe()

    def _apply_initial_state(self, st: dict) -> None:
        # freejoint root is joint 0 typically named "root"
        root = "root"
        if root not in self._joint_qpos_adr:
            raise RuntimeError("root freejoint not found")
        adr = self._joint_qpos_adr[root]
        dadr = self._joint_qvel_adr[root]
        if "root_pos" in st:
            p = np.asarray(st["root_pos"], dtype=np.float64).reshape(3)
            self.data.qpos[adr : adr + 3] = p
        if "root_quat_wxyz" in st:
            q = np.asarray(st["root_quat_wxyz"], dtype=np.float64).reshape(4)
            n = np.linalg.norm(q)
            if n < 1e-12:
                raise ValueError("root_quat_wxyz has zero norm")
            self.data.qpos[adr + 3 : adr + 7] = q / n
        if "root_lin_vel" in st:
            self.data.qvel[dadr : dadr + 3] = np.asarray(st["root_lin_vel"], dtype=np.float64).reshape(3)
        if "root_ang_vel" in st:
            self.data.qvel[dadr + 3 : dadr + 6] = np.asarray(st["root_ang_vel"], dtype=np.float64).reshape(3)
        hinge_map = dict(st.get("hinge_qpos") or {})
        if "joint_angles" in st:
            for k, v in st["joint_angles"].items():
                if k not in self._joint_qpos_adr:
                    raise KeyError(f"unknown joint in joint_angles: {k}")
                jtype = self._joint_type[k]
                a = self._joint_qpos_adr[k]
                if jtype == mujoco.mjtJoint.mjJNT_HINGE:
                    hinge_map[k] = float(v)
                elif jtype == mujoco.mjtJoint.mjJNT_BALL:
                    q = np.asarray(v, dtype=np.float64).reshape(4)
                    q = q / (np.linalg.norm(q) + 1e-12)
                    self.data.qpos[a : a + 4] = q
                else:
                    raise ValueError(f"joint_angles unsupported type for {k}")
        for k, v in hinge_map.items():
            if k not in self._joint_qpos_adr:
                raise KeyError(f"unknown hinge: {k}")
            if self._joint_type[k] != mujoco.mjtJoint.mjJNT_HINGE:
                raise ValueError(f"{k} is not a hinge")
            self.data.qpos[self._joint_qpos_adr[k]] = float(v)

    def observe(self) -> dict:
        """Participant observation — no hidden masses/geom sizes."""
        d = self.data
        m = self.model
        # Root freejoint
        adr = self._joint_qpos_adr["root"]
        dadr = self._joint_qvel_adr["root"]
        root_pos = d.qpos[adr : adr + 3].copy()
        root_quat = d.qpos[adr + 3 : adr + 7].copy()
        root_lin_vel = d.qvel[dadr : dadr + 3].copy()
        root_ang_vel = d.qvel[dadr + 3 : dadr + 6].copy()

        joint_angles = {}
        joint_vels = {}
        for name, a in self._joint_qpos_adr.items():
            if name == "root":
                continue
            jtype = self._joint_type[name]
            va = self._joint_qvel_adr[name]
            if jtype == mujoco.mjtJoint.mjJNT_HINGE:
                joint_angles[name] = float(d.qpos[a])
                joint_vels[name] = float(d.qvel[va])
            elif jtype == mujoco.mjtJoint.mjJNT_BALL:
                joint_angles[name] = d.qpos[a : a + 4].tolist()
                joint_vels[name] = d.qvel[va : va + 3].tolist()

        # Contact summary (no geom size leak)
        ncon = int(d.ncon)
        contact_forces = []
        floor_contact = False
        for i in range(ncon):
            c = d.contact[i]
            g1 = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, c.geom1) or ""
            g2 = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, c.geom2) or ""
            if "floor" in (g1, g2) or g1 == "floor" or g2 == "floor":
                floor_contact = True
            force = np.zeros(6)
            mujoco.mj_contactForce(m, d, i, force)
            contact_forces.append({
                "geom1": g1,
                "geom2": g2,
                "force_normal": float(force[0]),
                "pos": [float(c.pos[0]), float(c.pos[1]), float(c.pos[2])],
            })

        obs = {
            "time": float(d.time),
            "root_pos": root_pos.tolist(),
            "root_quat_wxyz": root_quat.tolist(),
            "root_lin_vel": root_lin_vel.tolist(),
            "root_ang_vel": root_ang_vel.tolist(),
            "joint_angles": joint_angles,
            "joint_vels": joint_vels,
            "contact": {
                "ncon": ncon,
                "floor_contact": floor_contact,
                "forces_summary": contact_forces[:32],  # cap
            },
            "ctrl": d.ctrl.copy().tolist(),
            "action_names": list(ACTION_NAMES),
            "image_path": self._last_image_path,
        }
        return obs

    def apply_torques(self, action, duration: float) -> dict:
        """Apply torque vector for `duration` seconds (clipped). Returns observe()."""
        action = np.asarray(action, dtype=np.float64).reshape(-1)
        if action.shape[0] != self.model.nu:
            raise ValueError(f"action dim {action.shape[0]} != nu={self.model.nu}")
        duration = float(duration)
        if duration < MIN_APPLY_DURATION:
            duration = MIN_APPLY_DURATION
        if duration > MAX_APPLY_DURATION:
            duration = MAX_APPLY_DURATION

        clipped = np.clip(action, self._torque_limits[:, 0], self._torque_limits[:, 1])
        self.data.ctrl[:] = clipped
        self._episode_actions.append({"action": clipped.tolist(), "duration": duration})

        dt = float(self.model.opt.timestep)
        n_steps = int(round(duration / dt))
        for _ in range(n_steps):
            mujoco.mj_step(self.model, self.data)
        return self.observe()

    def step(self) -> dict:
        """Single physics timestep with current ctrl."""
        mujoco.mj_step(self.model, self.data)
        return self.observe()

    def render_observation(self, path: Optional[str] = None) -> str:
        """Headless RGB via MuJoCo renderer (not Blender). Optional path to save PNG."""
        try:
            from PIL import Image
        except ImportError:
            Image = None
        w, h = 640, 480
        renderer = mujoco.Renderer(self.model, height=h, width=w)
        renderer.update_scene(self.data)
        pixels = renderer.render()
        renderer.close()
        if path is None:
            path = str(ROOT / "shots" / f"obs_{self.data.time:.3f}.png")
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        if Image is not None:
            Image.fromarray(pixels).save(path)
        else:
            # fallback write via mujoco if no PIL — still store path intent
            np.save(path + ".npy", pixels)
            path = path + ".npy"
        self._last_image_path = path
        return path

    # Convenience (still participant-safe)
    def clip_action(self, action) -> np.ndarray:
        action = np.asarray(action, dtype=np.float64).reshape(-1)
        return np.clip(action, self._torque_limits[:, 0], self._torque_limits[:, 1])

    @property
    def nu(self) -> int:
        return int(self.model.nu)

    @property
    def timestep(self) -> float:
        return float(self.model.opt.timestep)
