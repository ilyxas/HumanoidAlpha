"""Time-series diagnostics shared by every Xandra-Lite variant (controller x geometry).

Per 10 ms sample: COM ground projection, measured CoP (from contact normal forces), ZMP estimate
(COM - z_com/g * COM acceleration), and their signed margins to the support polygon (convex hull of
the sole corners that touch the ground; > 0 inside), per-foot normal force, contact state, actuator
force/limit ratio. Events: first time the ZMP leaves the support polygon, swing-foot unload
(< 12 % body weight) and lift-off, first actuator saturation per joint.
"""
from __future__ import annotations

import mujoco
import numpy as np

G = 9.81


def _hull(pts):
    pts = sorted(set(map(tuple, np.round(pts, 6))))
    if len(pts) < 3:
        return np.array(pts)

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower, upper = [], []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return np.array(lower[:-1] + upper[:-1])


def margin(point, hull):
    """Signed distance from point to a convex polygon (counter-clockwise); > 0 inside."""
    if len(hull) < 3:
        return -np.inf
    best = np.inf
    for i in range(len(hull)):
        a, b = hull[i], hull[(i + 1) % len(hull)]
        e = b - a
        n = np.array([-e[1], e[0]]) / np.linalg.norm(e)  # inward normal for CCW
        best = min(best, float(np.dot(point - a, n)))
    return best


class Diag:
    def __init__(self, m, ctrl, every_s=0.01):
        self.m, self.c = m, ctrl
        self.every = max(1, int(round(every_s / m.opt.timestep)))
        self.k = 0
        self.prev_v = None
        self.rows = []
        self.events = {}
        self.weight = float(m.body_subtreemass[ctrl.torso]) * G
        self.jc = np.zeros((3, m.nv))

    def _event(self, name, t, **kw):
        if name not in self.events:
            self.events[name] = dict(t=round(float(t), 3), **kw)

    def step(self, d):
        m, c = self.m, self.c
        mujoco.mj_jacSubtreeCom(m, d, self.jc, c.torso)
        v = (self.jc @ d.qvel)[:2]
        raw = np.zeros(2) if self.prev_v is None else (v - self.prev_v) / m.opt.timestep
        self.prev_v = v
        a = m.opt.timestep / 0.02  # ~20 ms low-pass: contact impacts make raw differences meaningless
        self.acc = raw if not hasattr(self, "acc") else (1 - a) * self.acc + a * raw
        acc = self.acc
        self.k += 1
        com = d.subtree_com[c.torso]
        # contacts of the soles
        f6 = np.zeros(6)
        force = [0.0, 0.0]
        cop_num, cop_den = np.zeros(2), 0.0
        corners = []
        for i in range(d.ncon):
            con = d.contact[i]
            for k in range(2):
                if c.soles[k] in (con.geom1, con.geom2) and con.dist < 0.003:
                    mujoco.mj_contactForce(m, d, i, f6)
                    force[k] += f6[0]
                    cop_num += con.pos[:2] * f6[0]
                    cop_den += f6[0]
        on = [force[k] > 1.0 for k in range(2)]
        for k in range(2):
            if on[k]:
                g = c.soles[k]
                R, ctr = d.geom_xmat[g].reshape(3, 3), d.geom_xpos[g]
                sx, sy, sz = m.geom_size[g]
                corners += [(ctr + R @ np.array([a * sx, b * sy, -sz]))[:2] for a in (-1, 1) for b in (-1, 1)]
        hull = _hull(np.array(corners)) if corners else np.zeros((0, 2))
        cop = cop_num / cop_den if cop_den > 1e-6 else np.full(2, np.nan)
        zmp = com[:2] - com[2] / G * acc
        m_com, m_cop, m_zmp = margin(com[:2], hull), (margin(cop, hull) if cop_den > 1e-6 else np.nan), margin(zmp, hull)
        t = float(d.time)
        if np.isfinite(m_zmp) and m_zmp < 0 and self.k > 50:
            self._event("zmp_first_outside_support", t, margin_cm=round(m_zmp * 100, 2))
        if np.isfinite(m_com) and m_com < 0:
            self._event("com_first_outside_support", t)
        for k in range(2):
            side = "lr"[k]
            if force[k] < 0.12 * self.weight and force[1 - k] > 0.5 * self.weight:
                self._event(f"unload_{side}", t)
            if not on[k] and on[1 - k]:
                self._event(f"liftoff_{side}", t)
        ratio = np.abs(d.actuator_force[c.act]) / np.array([m.actuator_forcerange[i, 1] if m.actuator_forcelimited[i] else m.actuator_ctrlrange[i, 1] for i in c.act])
        for a, r in zip(getattr(c, "act_names", None) or [m.actuator(i).name for i in c.act], ratio):
            if r >= 0.999:
                self._event(f"saturated_{a}", t)
        if self.k % self.every == 0:
            self.rows.append(dict(t=round(t, 3), com=np.round(com[:2], 4).tolist(), cop=np.round(cop, 4).tolist(),
                                  zmp=np.round(zmp, 4).tolist(), m_com=round(m_com * 100, 2) if np.isfinite(m_com) else None,
                                  m_cop=round(m_cop * 100, 2) if np.isfinite(m_cop) else None,
                                  m_zmp=round(m_zmp * 100, 2) if np.isfinite(m_zmp) else None,
                                  F=[round(force[0], 1), round(force[1], 1)],
                                  sat=[a for a, r in zip([m.actuator(i).name for i in c.act], ratio) if r >= 0.999]))

    def summary(self):
        mz = [r["m_zmp"] for r in self.rows if r["m_zmp"] is not None]
        mc = [r["m_cop"] for r in self.rows if r["m_cop"] is not None]
        return dict(events=self.events, min_zmp_margin_cm=min(mz) if mz else None, min_cop_margin_cm=min(mc) if mc else None,
                    samples=len(self.rows))
