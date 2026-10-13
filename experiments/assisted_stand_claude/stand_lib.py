"""Assisted-stand investigation helpers. The controller and the flat-sole variant live in
physics/stand_v2.py (also used by the stand-v2 / debug-v2 launcher modes)."""
from __future__ import annotations

import sys
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "physics"))
from stand_v2 import (MODEL, StanceID, actuator_dofs, equilibrium_seed, floor_fz,  # noqa: E402,F401
                      load_flat_sole_model, tilt_deg)


def load_model(variant: str = "mesh") -> mujoco.MjModel:
    """'mesh' = repository model as is; 'flat_sole' = floor touches a box sole instead of the foot hull."""
    if variant == "mesh":
        return mujoco.MjModel.from_xml_path(str(MODEL))
    if variant == "flat_sole":
        return load_flat_sole_model()
    raise ValueError(variant)


def pitch_back_deg(d, body) -> float:
    """Sagittal lean of the body z-axis; + = backward (+Y; she faces -Y in MuJoCo)."""
    r = d.xmat[body].reshape(3, 3)
    return float(np.degrees(np.arctan2(r[1, 2], r[2, 2])))
