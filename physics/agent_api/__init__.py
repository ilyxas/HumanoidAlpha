"""Humanoid Alpha — Participant Agent API (torque-only, no bpy / raw mjData)."""
from .env import HumanoidEnv, ACTION_DIM, ACTION_NAMES, TORQUE_LIMITS, MAX_APPLY_DURATION

__all__ = [
    "HumanoidEnv",
    "ACTION_DIM",
    "ACTION_NAMES",
    "TORQUE_LIMITS",
    "MAX_APPLY_DURATION",
]
