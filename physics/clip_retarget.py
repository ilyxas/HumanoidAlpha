"""Animation clips (Xandra_Animated.glb) -> physical model targets. Experimental (anim-physics).

Inverse of bridge_full/mapping_full + console/observation viewers:
  viewer:      q_three_world(bone) = QC * q_blender_world(bone),  QC = Rx(-90 deg)
  mapping:     R_blender_world(bone) = R_body_world @ R_rest(bone)        (calibration_rest.json)
  so           q_body_world = QC^-1 * q_three_world(lead bone) * conj(q_rest(lead bone))
Joint coordinates: every MuJoCo body has identity rest orientation, so a ball joint is
conj(q_parent) * q_child; hinges are fitted to that relative rotation (two hinges in a body are
solved together). Targets are clamped to the model's joint limits.

Root: pelvis bone world position (Y-up -> Z-up; the GLB floor is y = 0) and the pelvis body
orientation. Per frame it also stores which feet are on the floor in the clip (heel or toe bone near
its bind height) and the clip COM relative to those feet.
"""
from __future__ import annotations

import json
import struct
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
GLB = ROOT / "assets" / "Xandra_Animated.glb"
CALIB = Path(__file__).resolve().parent / "bridge_full" / "calibration_rest.json"
QC_INV = np.array([np.cos(np.pi / 4), np.sin(np.pi / 4), 0.0, 0.0])  # Rx(+90 deg), wxyz
STANCE_DZ = 0.025  # m: heel/toe bone this close to its bind height counts as support


def qmul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2])


def qconj(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])


def qnorm(q):
    q = np.asarray(q, float) / np.linalg.norm(q)
    return q if q[0] >= 0 else -q


def qslerp(a, b, t):
    a, b = qnorm(a), np.asarray(b, float) / np.linalg.norm(b)
    d = float(np.dot(a, b))
    if d < 0:
        b, d = -b, -d
    if d > 0.9995:
        return qnorm(a + t * (b - a))
    th = np.arccos(min(1.0, d))
    return qnorm((np.sin((1 - t) * th) * a + np.sin(t * th) * b) / np.sin(th))


def qaxis(axis, ang):
    axis = np.asarray(axis, float)
    return np.r_[np.cos(ang / 2), np.sin(ang / 2) * axis / np.linalg.norm(axis)]


def qrotvec(q):
    q = qnorm(q)
    s = np.linalg.norm(q[1:])
    return np.zeros(3) if s < 1e-12 else q[1:] / s * 2 * np.arctan2(s, q[0])


def qmat(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class Glb:
    """Minimal glTF 2.0 binary reader: node hierarchy and sampled animation channels."""

    def __init__(self, path: Path = GLB):
        raw = path.read_bytes()
        n = struct.unpack_from("<I", raw, 12)[0]
        self.j = json.loads(raw[20:20 + n])
        off = 20 + n
        bn = struct.unpack_from("<I", raw, off)[0]
        self.bin = raw[off + 8: off + 8 + bn]
        self.nodes = self.j["nodes"]
        self.parent = {}
        for i, nd in enumerate(self.nodes):
            for c in nd.get("children", []):
                self.parent[c] = i
        self.by_name = {nd.get("name"): i for i, nd in enumerate(self.nodes)}
        self.anims = {a["name"]: a for a in self.j.get("animations", [])}

    def accessor(self, i):
        a = self.j["accessors"][i]
        bv = self.j["bufferViews"][a["bufferView"]]
        k = {"SCALAR": 1, "VEC3": 3, "VEC4": 4}[a["type"]]
        if a["componentType"] != 5126:
            raise ValueError("only float accessors are supported")
        start = bv.get("byteOffset", 0) + a.get("byteOffset", 0)
        arr = np.frombuffer(self.bin, dtype="<f4", count=a["count"] * k, offset=start)
        return arr.reshape(a["count"], k) if k > 1 else arr

    def channels(self, clip):
        """{node: {path: (times, values, interpolation)}}"""
        a = self.anims[clip]
        out = {}
        for ch in a["channels"]:
            s = a["samplers"][ch["sampler"]]
            out.setdefault(ch["target"]["node"], {})[ch["target"]["path"]] = (
                self.accessor(s["input"]), self.accessor(s["output"]), s.get("interpolation", "LINEAR"))
        return out

    @staticmethod
    def sample(track, t, rot):
        times, vals, interp = track
        if t <= times[0]:
            return np.array(vals[0], float)
        if t >= times[-1]:
            return np.array(vals[-1], float)
        i = int(np.searchsorted(times, t) - 1)
        if interp == "STEP":
            return np.array(vals[i], float)
        u = (t - times[i]) / (times[i + 1] - times[i])
        if rot:  # glTF xyzw
            q = qslerp(np.r_[vals[i][3], vals[i][:3]], np.r_[vals[i + 1][3], vals[i + 1][:3]], u)
            return np.r_[q[1:], q[0]]
        return (1 - u) * np.asarray(vals[i], float) + u * np.asarray(vals[i + 1], float)

    def world(self, names, chans=None, t=0.0):
        """World (Y-up) rotation wxyz and position of named nodes at time t (bind pose if chans is None)."""
        cache = {}

        def local(i):
            nd = self.nodes[i]
            tr = chans.get(i, {}) if chans else {}
            r = self.sample(tr["rotation"], t, True) if "rotation" in tr else np.array(nd.get("rotation", [0, 0, 0, 1.0]))
            p = self.sample(tr["translation"], t, False) if "translation" in tr else np.array(nd.get("translation", [0, 0, 0.0]))
            s = self.sample(tr["scale"], t, False) if "scale" in tr else np.array(nd.get("scale", [1, 1, 1.0]))
            return np.r_[r[3], r[:3]], p, s

        def get(i):
            # (world rotation, world position, accumulated uniform scale); the armature node
            # carries scale 0.01 (joint data in cm), which must reach every descendant.
            if i in cache:
                return cache[i]
            q, p, s = local(i)
            if i in self.parent:
                pq, pp, ps = get(self.parent[i])
                res = (qmul(pq, q), pp + ps * (qmat(pq) @ p), ps * float(s[0]))
            else:
                res = (q, p, float(s[0]))
            cache[i] = res
            return res

        out = {}
        for n in names:
            q, p, _s = get(self.by_name[n])
            out[n] = (qnorm(q), p)
        return out


def _calib():
    c = json.loads(CALIB.read_text())
    return c["body_to_bone"], {b: np.array(v["quat_rest_wxyz"], float) for b, v in c["bones"].items()}


class Retargeter:
    def __init__(self, model: mujoco.MjModel, glb: Glb | None = None):
        self.m = model
        self.d = mujoco.MjData(model)
        self.glb = glb or Glb()
        self.body_to_bone, self.rest = _calib()
        self.bones = sorted(set(self.body_to_bone.values()))
        self.joints = [j for j in range(model.njnt) if model.jnt_type[j] != mujoco.mjtJoint.mjJNT_FREE]
        # joints grouped by body, in MJCF order (MuJoCo composes them in this order)
        self.body_joints = {}
        for j in self.joints:
            self.body_joints.setdefault(int(model.jnt_bodyid[j]), []).append(j)
        self.soles = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, n) for n in ("sole_l", "sole_r")]
        self.torso = model.body("torso_upper").id
        try:
            man = json.loads((ROOT / "assets" / "Xandra_Animated.manifest.json").read_text())
            self.loops = {k: bool(v.get("loop")) for k, v in man.get("clips", {}).items()}
        except (OSError, ValueError):
            self.loops = {n: n in ("Idle", "Walk_Fwd", "Run_Fwd", "Walk_Back") for n in self.glb.anims}

    def body_quats(self, chans, t):
        w = self.glb.world(self.bones, chans, t)
        out = {}
        for body, bone in self.body_to_bone.items():
            q3, _ = w[bone]
            out[body] = qnorm(qmul(qmul(QC_INV, q3), qconj(self.rest[bone])))
        pelvis_y = w["pelvis"][1]
        return out, np.array([pelvis_y[0], -pelvis_y[2], pelvis_y[1]])

    def qpos_from(self, bq, pelvis_pos):
        m = self.m
        q = m.qpos0.copy()
        q[0:3] = pelvis_pos
        q[3:7] = bq["pelvis"]
        for b, js in self.body_joints.items():
            name = m.body(b).name
            parent = m.body(int(m.body_parentid[b])).name
            rel = qnorm(qmul(qconj(bq[parent]), bq[name]))
            if m.jnt_type[js[0]] == mujoco.mjtJoint.mjJNT_BALL:
                rv = qrotvec(rel)
                lim = m.jnt_range[js[0], 1]
                ang = np.linalg.norm(rv)
                if m.jnt_limited[js[0]] and ang > lim:
                    rv *= lim / ang
                a = m.jnt_qposadr[js[0]]
                q[a:a + 4] = qnorm(np.r_[np.cos(np.linalg.norm(rv) / 2), np.sinc(np.linalg.norm(rv) / 2 / np.pi) * rv / 2])
            else:
                axes = [m.jnt_axis[j] for j in js]
                th = np.array([2 * np.arctan2(np.dot(rel[1:], ax), rel[0]) for ax in axes])
                for _ in range(20):  # Gauss-Newton on the composed hinge rotation
                    cur = np.r_[1.0, 0, 0, 0]
                    for ax, a in zip(axes, th):
                        cur = qmul(cur, qaxis(ax, a))
                    err = qrotvec(qmul(qconj(cur), rel))
                    if np.linalg.norm(err) < 1e-6:
                        break
                    jac = np.zeros((3, len(axes)))
                    for i in range(len(axes)):
                        dth = th.copy()
                        dth[i] += 1e-5
                        c2 = np.r_[1.0, 0, 0, 0]
                        for ax, a in zip(axes, dth):
                            c2 = qmul(c2, qaxis(ax, a))
                        jac[:, i] = (qrotvec(qmul(qconj(cur), c2))) / 1e-5
                    th += np.linalg.lstsq(jac, err, rcond=None)[0]
                for j, a in zip(js, th):
                    lo, hi = m.jnt_range[j] if m.jnt_limited[j] else (-np.inf, np.inf)
                    q[m.jnt_qposadr[j]] = np.clip(a, lo, hi)
        return q

    def _sole_corners(self):
        m, d = self.m, self.d
        out = []
        for g in self.soles:
            rot, ctr = d.geom_xmat[g].reshape(3, 3), d.geom_xpos[g]
            sx, sy, sz = m.geom_size[g]
            out.append(np.array([ctr + rot @ np.array([a * sx, b * sy, -sz]) for a in (-1, 1) for b in (-1, 1)]))
        return out

    def clip(self, name, fps=60.0):
        """Retargeted clip: dict(t, qpos, qvel, stance[n,2], com_off[n,2], duration, loop?)"""
        chans = self.glb.channels(name)
        dur = max(float(tr[0][-1]) for ch in chans.values() for tr in ch.values())
        ts = np.arange(0.0, dur + 1e-9, 1.0 / fps)
        qs = np.array([self.qpos_from(*self.body_quats(chans, t)) for t in ts])
        # Support phases from the clip itself (GLB floor is y = 0): a foot is on the floor when its
        # heel (foot bone) or toe (ball bone) is within STANCE_DZ of its bind-pose height.
        bind = self.glb.world(["foot_l", "ball_l", "foot_r", "ball_r"])
        stance = np.zeros((len(ts), 2), bool)
        com_off = np.zeros((len(ts), 2))
        torso_tilt = np.zeros(len(ts))
        for i, (t, q) in enumerate(zip(ts, qs)):
            w = self.glb.world(["foot_l", "ball_l", "foot_r", "ball_r"], chans, t)
            for k, side in enumerate("lr"):
                stance[i, k] = min(w[f"foot_{side}"][1][1] - bind[f"foot_{side}"][1][1],
                                   w[f"ball_{side}"][1][1] - bind[f"ball_{side}"][1][1]) < STANCE_DZ
            self.d.qpos[:] = q
            mujoco.mj_kinematics(self.m, self.d)
            mujoco.mj_comPos(self.m, self.d)
            c = self._sole_corners()
            feet = [c[k][:, :2].mean(0) for k in range(2) if stance[i, k]] or [c[k][:, :2].mean(0) for k in range(2)]
            com_off[i] = self.d.subtree_com[1, :2] - np.mean(feet, axis=0)
            torso_tilt[i] = np.degrees(np.arccos(np.clip(self.d.xmat[self.torso].reshape(3, 3)[2, 2], -1, 1)))
        qv = np.zeros((len(ts), self.m.nv))
        for i in range(1, len(ts) - 1):
            mujoco.mj_differentiatePos(self.m, qv[i], 2.0 / fps, qs[i - 1], qs[i + 1])
        return dict(name=name, t=ts, qpos=qs, qvel=qv, stance=stance, com_off=com_off, torso_tilt=torso_tilt,
                    duration=dur, fps=fps, loop=bool(self.loops.get(name, False)))

    def all_clips(self, fps=30.0):
        return {name: self.clip(name, fps) for name in self.glb.anims}
