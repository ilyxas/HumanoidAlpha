"""Full-body MuJoCo → Blender pose extraction (world-orientation drive).

Method (MEASURED):
  At MuJoCo rest, every physical body xquat == identity.
  Blender bones have non-identity rest frames (UE +Y along bone).
  Alignment per bone: A = R_bone_rest_world  (since R_mj_rest = I)
  Runtime: R_bone_world_desired = R_mj_body_world @ A

Integrity (ENGINEERING, 2026-09-28):
  Physical links are coarse (torso_lower / torso_upper / head). Visual spine has
  spine_01..05 + neck_01/02. Drive intermediates so each visual bone along a
  physical link shares that link's world orientation (no rest-stuck crumple):
    spine_01, spine_02  ← torso_lower
    spine_03, spine_04, spine_05  ← torso_upper
    neck_01, neck_02  ← slerp(torso_upper, head; arc-length fraction)
    head  ← head
  Clavicles stay rest-local under spine_05 (welded in MJCF; not torque-driven).

Physics remains SoT: we never write back to MuJoCo / nudge qpos for looks.
"""
from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
CALIB_PATH = Path(__file__).resolve().parent / "calibration_rest.json"

# Physical body → primary visual lead bone (orientation source)
BODY_TO_BONE = {
    "pelvis": "pelvis",
    "torso_lower": "spine_01",
    "torso_upper": "spine_05",
    "head": "head",
    "upperarm_l": "upperarm_l",
    "upperarm_r": "upperarm_r",
    "lowerarm_l": "lowerarm_l",
    "lowerarm_r": "lowerarm_r",
    "hand_l": "hand_l",
    "hand_r": "hand_r",
    "thigh_l": "thigh_l",
    "thigh_r": "thigh_r",
    "calf_l": "calf_l",
    "calf_r": "calf_r",
    "foot_l": "foot_l",
    "foot_r": "foot_r",
}

# Extra visual bones driven for chain continuity (not 1:1 physical bodies).
# source: physical body name OR ("slerp", body_a, body_b, t)
DERIVED_BONE_SOURCES = {
    "spine_02": "torso_lower",
    "spine_03": "torso_upper",
    "spine_04": "torso_upper",
    # spine_05 already in BODY_TO_BONE ← torso_upper
    "neck_01": ("slerp", "torso_upper", "head", 0.50),
    "neck_02": ("slerp", "torso_upper", "head", 0.77),
    # Match the welded physical clavicles: shoulder rotation must not move its
    # visual anchor away from the collision body / physical shoulder joint.
    "clavicle_l": "torso_upper",
    "clavicle_r": "torso_upper",
}

# Better position-comparison anchors (MEASURED rest coincidence)
POSITION_ANCHORS = {
    # bone → mj body (or geom) for positional integrity metrics
    "pelvis": ("body", "pelvis"),
    "spine_01": ("body", "torso_lower"),
    "spine_03": ("body", "torso_upper"),
    "spine_05": ("body", "head"),  # neck joint
    "head": ("geom", "head_geom"),
    "upperarm_l": ("body", "upperarm_l"),
    "upperarm_r": ("body", "upperarm_r"),
    "lowerarm_l": ("body", "lowerarm_l"),
    "lowerarm_r": ("body", "lowerarm_r"),
    "hand_l": ("body", "hand_l"),
    "hand_r": ("body", "hand_r"),
    "thigh_l": ("body", "thigh_l"),
    "thigh_r": ("body", "thigh_r"),
    "calf_l": ("body", "calf_l"),
    "calf_r": ("body", "calf_r"),
    "foot_l": ("body", "foot_l"),
    "foot_r": ("body", "foot_r"),
}

# Parents before children so Blender pb.matrix bak-solve is stable.
BONES_APPLY_ORDER = [
    "pelvis",
    "spine_01",
    "spine_02",
    "spine_03",
    "spine_04",
    "spine_05",
    "neck_01",
    "neck_02",
    "head",
    "thigh_l",
    "thigh_r",
    "calf_l",
    "calf_r",
    "foot_l",
    "foot_r",
    "clavicle_l",
    "clavicle_r",
    "upperarm_l",
    "upperarm_r",
    "lowerarm_l",
    "lowerarm_r",
    "hand_l",
    "hand_r",
]

# Still unmapped deform / twist / corrective (documented)
UNMAPPED_VISUAL = [
    "ball_l", "ball_r",
    "ik_foot_root", "ik_hand_root",
]


def quat_normalize(q):
    w, x, y, z = q
    n = math.sqrt(w * w + x * x + y * y + z * z) or 1.0
    return (w / n, x / n, y / n, z / n)


def quat_conj(q):
    w, x, y, z = q
    return (w, -x, -y, -z)


def quat_mul(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return (
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    )


def quat_dot(q1, q2):
    return q1[0] * q2[0] + q1[1] * q2[1] + q1[2] * q2[2] + q1[3] * q2[3]


def quat_slerp(q1, q2, t):
    """Unit-quaternion slerp; t in [0,1]."""
    qa = quat_normalize(q1)
    qb = quat_normalize(q2)
    dot = quat_dot(qa, qb)
    if dot < 0.0:
        qb = (-qb[0], -qb[1], -qb[2], -qb[3])
        dot = -dot
    if dot > 0.9995:
        # linear fallback
        r = (
            qa[0] + t * (qb[0] - qa[0]),
            qa[1] + t * (qb[1] - qa[1]),
            qa[2] + t * (qb[2] - qa[2]),
            qa[3] + t * (qb[3] - qa[3]),
        )
        return quat_normalize(r)
    dot = min(1.0, max(-1.0, dot))
    theta_0 = math.acos(dot)
    sin_0 = math.sin(theta_0)
    theta = theta_0 * t
    s0 = math.sin(theta_0 - theta) / sin_0
    s1 = math.sin(theta) / sin_0
    return (
        s0 * qa[0] + s1 * qb[0],
        s0 * qa[1] + s1 * qb[1],
        s0 * qa[2] + s1 * qb[2],
        s0 * qa[3] + s1 * qb[3],
    )


def quat_to_R(q):
    w, x, y, z = quat_normalize(q)
    return (
        (1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
        (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
        (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)),
    )


def R_to_quat(R):
    r00, r01, r02 = R[0]
    r10, r11, r12 = R[1]
    r20, r21, r22 = R[2]
    t = r00 + r11 + r22
    if t > 0:
        s = 0.5 / math.sqrt(t + 1.0)
        w = 0.25 / s
        x = (r21 - r12) * s
        y = (r02 - r20) * s
        z = (r10 - r01) * s
    else:
        if r00 > r11 and r00 > r22:
            s = 2.0 * math.sqrt(1.0 + r00 - r11 - r22)
            w = (r21 - r12) / s
            x = 0.25 * s
            y = (r01 + r10) / s
            z = (r02 + r20) / s
        elif r11 > r22:
            s = 2.0 * math.sqrt(1.0 + r11 - r00 - r22)
            w = (r02 - r20) / s
            x = (r01 + r10) / s
            y = 0.25 * s
            z = (r12 + r21) / s
        else:
            s = 2.0 * math.sqrt(1.0 + r22 - r00 - r11)
            w = (r10 - r01) / s
            x = (r02 + r20) / s
            y = (r12 + r21) / s
            z = 0.25 * s
    q = quat_normalize((w, x, y, z))
    if q[0] < 0:
        q = (-q[0], -q[1], -q[2], -q[3])
    return q


def matmul(A, B):
    return tuple(
        tuple(sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3))
        for i in range(3)
    )


def quat_rotate_R(q_mj, R_rest):
    """R_desired = R(q_mj) @ R_rest → quat."""
    return R_to_quat(matmul(quat_to_R(q_mj), R_rest))


def angular_error_rad(q_a, q_b):
    """Geodesic angle between unit quaternions."""
    qa = quat_normalize(q_a)
    qb = quat_normalize(q_b)
    dot = abs(qa[0] * qb[0] + qa[1] * qb[1] + qa[2] * qb[2] + qa[3] * qb[3])
    dot = min(1.0, max(0.0, dot))
    return 2.0 * math.acos(dot)


_calib_cache = None


def load_calibration(path: Path | str | None = None) -> dict:
    global _calib_cache
    p = Path(path) if path else CALIB_PATH
    if _calib_cache is not None and path is None:
        return _calib_cache
    data = json.loads(p.read_text())
    if path is None:
        _calib_cache = data
    return data


def clear_calibration_cache():
    global _calib_cache
    _calib_cache = None


def _mj_quat_for_source(source, body_xquat: dict):
    if isinstance(source, str):
        return tuple(body_xquat[source])
    kind, a, b, t = source
    assert kind == "slerp"
    return quat_slerp(tuple(body_xquat[a]), tuple(body_xquat[b]), float(t))


def extract_full_pose(
    obs: dict,
    body_xpos: dict | None = None,
    body_xquat: dict | None = None,
    *,
    calibration: dict | None = None,
) -> dict[str, Any]:
    """Build full visual pose: root translate + world-orientation targets.

    Output schema consumed by bridge_full/apply_full_blender.py:
      mode: "body_world_aligned"
      root_pos, root_quat_wxyz
      bone_world_quat: {bone: quat_wxyz desired WORLD rotation}
      bone_world_pos:  {bone: xyz} diagnostic (MuJoCo body xpos for leads)
      apply_order: list
      integrity_bones: derived chain bones included
    """
    calib = calibration or load_calibration()
    bone_info = calib["bones"]

    if body_xquat is None:
        raise ValueError("extract_full_pose requires body_xquat from HumanoidDiag")

    bone_world_quat = {}
    bone_world_pos = {}

    # Primary physical leads
    for body, bone in BODY_TO_BONE.items():
        if body not in body_xquat:
            continue
        if bone not in bone_info:
            continue
        q_mj = tuple(body_xquat[body])
        R_rest = tuple(tuple(row) for row in bone_info[bone]["R_rest_world"])
        q_des = quat_rotate_R(q_mj, R_rest)
        bone_world_quat[bone] = list(q_des)
        if body_xpos and body in body_xpos:
            bone_world_pos[bone] = list(body_xpos[body])

    # Derived intermediates for chain continuity
    for bone, source in DERIVED_BONE_SOURCES.items():
        if bone not in bone_info:
            continue
        try:
            q_mj = _mj_quat_for_source(source, body_xquat)
        except KeyError:
            continue
        R_rest = tuple(tuple(row) for row in bone_info[bone]["R_rest_world"])
        q_des = quat_rotate_R(q_mj, R_rest)
        bone_world_quat[bone] = list(q_des)

    pose = {
        "mode": "body_world_aligned",
        "root_pos": list(obs["root_pos"]),
        "root_quat_wxyz": list(obs["root_quat_wxyz"]),
        "apply_root_quat": True,
        "apply_root_translate": True,
        "bone_world_quat": bone_world_quat,
        "bone_world_pos": bone_world_pos,
        "apply_order": [b for b in BONES_APPLY_ORDER if b in bone_world_quat],
        "body_to_bone": dict(BODY_TO_BONE),
        "derived_bone_sources": {
            k: (list(v) if isinstance(v, tuple) else v)
            for k, v in DERIVED_BONE_SOURCES.items()
        },
        "position_anchors": {k: list(v) for k, v in POSITION_ANCHORS.items()},
        "calibration_method": calib.get("method"),
        "bones": {},
        "joint_angles_diag": obs.get("joint_angles"),
        "integrity_bones": [b for b in DERIVED_BONE_SOURCES if b in bone_world_quat],
    }
    return pose
