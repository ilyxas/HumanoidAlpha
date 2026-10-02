"""HumanoidDiag — full-state developer access."""
from __future__ import annotations
from typing import Any
import numpy as np
import mujoco
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from agent_api.env import HumanoidEnv


class HumanoidDiag:
    """Wraps HumanoidEnv with developer-only introspection."""

    def __init__(self, env: HumanoidEnv | None = None, **kwargs):
        self.env = env or HumanoidEnv(**kwargs)

    @property
    def model(self):
        return self.env.model

    @property
    def data(self):
        return self.env.data

    def full_qpos(self) -> np.ndarray:
        return self.env.data.qpos.copy()

    def full_qvel(self) -> np.ndarray:
        return self.env.data.qvel.copy()

    def energies(self) -> dict[str, float]:
        d = self.env.data
        return {
            "kinetic": float(d.energy[0]) if d.energy is not None else float("nan"),
            "potential": float(d.energy[1]) if d.energy is not None else float("nan"),
            "total": float(d.energy[0] + d.energy[1]) if d.energy is not None else float("nan"),
        }

    def body_xpos(self) -> dict[str, list]:
        out = {}
        for i in range(self.model.nbody):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, i)
            out[name or f"body_{i}"] = self.data.xpos[i].tolist()
        return out

    def body_xquat(self) -> dict[str, list]:
        out = {}
        for i in range(self.model.nbody):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, i)
            out[name or f"body_{i}"] = self.data.xquat[i].tolist()
        return out

    def com(self) -> list:
        # subtree com of pelvis / root body
        # data.subtree_com[1] often pelvis if body 0 is world
        for i in range(self.model.nbody):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, i)
            if name == "pelvis":
                return self.data.subtree_com[i].tolist()
        return self.data.subtree_com[1].tolist()

    def masses(self) -> dict[str, float]:
        out = {}
        for i in range(self.model.nbody):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, i)
            out[name or f"body_{i}"] = float(self.model.body_mass[i])
        return out

    def snapshot(self) -> dict[str, Any]:
        return {
            "qpos": self.full_qpos().tolist(),
            "qvel": self.full_qvel().tolist(),
            "energies": self.energies(),
            "body_xpos": self.body_xpos(),
            "com": self.com(),
            "time": float(self.data.time),
            "ncon": int(self.data.ncon),
        }
