"""Xandra-Lite, path 1-QP: the same skill planner as servo_control, low level = one QP per tick.

Decision variables x = [qacc (nv), tau (nu), f (3 per sole corner in contact)].
  equality     M qacc + c = S^T tau + sum Jc^T f           (rigid-body dynamics)
  inequality   -limit <= tau <= limit                         (motor force/torque limits)
               0 <= fz <= 2000, |fx|, |fy| <= MU fz            (friction pyramid per corner)
               corner forces >= 0  =>  the centre of pressure cannot leave the sole
  objective    weighted tasks on qacc: stance soles fixed (soft, high weight), swing-sole path,
               COM (x, y), torso height, upright torso, shoulder, weak posture; tiny regularization.
Motors are torque motors (model built with servo=False). No external forces, no state writes
except the logged reset seed (inherited).

Diagnostics (per Codex/Ilya contract): solver status / infeasibility, active constraints (motor
limits, friction, zero-force corners = CoP on the sole edge), task residuals, the CoP the QP
chose, and the "required" CoP = ZMP of the COM-task acceleration before any limit; events mark
the first time each of them hits.
"""
from __future__ import annotations

import sys
from pathlib import Path

import mujoco
import numpy as np

# osqp/scipy live in an isolated folder of the experiment branch (approved by Ilya, 2026-10-14);
# appended, so numpy and mujoco still come from the project venv.
_DEPS = Path(__file__).resolve().parents[2] / ".lite_deps"
if str(_DEPS) not in sys.path:
    sys.path.append(str(_DEPS))
import osqp  # noqa: E402
import scipy.sparse as sp  # noqa: E402

from .servo_control import ServoController  # noqa: E402

MU = 0.8          # friction used by the controller (model: 1.0)
G = 9.81


def _hull_margin(point, pts):
    """Signed distance of point to the convex hull of pts (>0 inside)."""
    pts = sorted(set(map(tuple, np.round(np.asarray(pts), 6))))
    if len(pts) < 3:
        return -np.inf

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lo, up = [], []
    for p in pts:
        while len(lo) >= 2 and cross(lo[-2], lo[-1], p) <= 0:
            lo.pop()
        lo.append(p)
    for p in reversed(pts):
        while len(up) >= 2 and cross(up[-2], up[-1], p) <= 0:
            up.pop()
        up.append(p)
    h = np.array(lo[:-1] + up[:-1])
    best = np.inf
    for i in range(len(h)):
        e = h[(i + 1) % len(h)] - h[i]
        n = np.array([-e[1], e[0]]) / np.linalg.norm(e)
        best = min(best, float(np.dot(np.asarray(point) - h[i], n)))
    return best


class QPController(ServoController):
    def __init__(self, m: mujoco.MjModel):
        super().__init__(m, plan_every=1)
        self.nv, self.nu = m.nv, len(self.act)
        self.S = np.zeros((self.nu, self.nv))
        for i, v in enumerate(self.act_v):
            self.S[i, v] = 1.0
        self.M = np.zeros((self.nv, self.nv))
        self.tau = np.zeros(self.nu)
        self.qp_events = {}
        self.qp_rows = []
        self.qp_stats = dict(solves=0, not_solved=0, tau_active_steps=0, friction_active_steps=0, edge_steps=0)
        self.jc = np.zeros((3, self.nv))

    def reset(self, d, joint_noise=0.0, seed=0):
        super().reset(d, joint_noise, seed)
        d.ctrl[:] = 0.0  # torque motors
        self.tau[:] = 0.0

    def _event(self, name, t, **kw):
        if name not in self.qp_events:
            self.qp_events[name] = dict(t=round(float(t), 3), **kw)

    # -------------------------------------------------------------- the QP
    def _actuate(self, d, stance, swing, com_t, torso_z, shoulder):
        m, nv, nu = self.m, self.nv, self.nu
        mujoco.mj_fullM(m, d, self.M)
        c = d.qfrc_bias.copy()
        rows_J, rows_b, rows_w = [], [], []

        def task(J, b, w):
            rows_J.append(np.atleast_2d(J))
            rows_b.append(np.atleast_1d(b))
            rows_w.append(np.full(len(np.atleast_1d(b)), w))

        # contact points: the lowest sole corners of each stance foot
        cpts, cfoot = [], []
        for k in range(2):
            if not stance[k]:
                continue
            g = self.soles[k]
            R, ctr = d.geom_xmat[g].reshape(3, 3), d.geom_xpos[g]
            sx, sy, sz = m.geom_size[g]
            cs = [ctr + R @ np.array([a * sx, b * sy, -sz]) for a in (-1, 1) for b in (-1, 1)]
            zmin = min(p[2] for p in cs)
            for p in cs:
                if p[2] <= zmin + 0.004:
                    cpts.append(p)
                    cfoot.append(k)
            # stance sole stays put; an edge-standing sole is rolled flat
            p = d.site_xpos[self.sole_sites[k]]
            mujoco.mj_jac(m, d, self.jp, self.jr, p, self.feet[k])
            jpd, jrd = np.zeros((3, nv)), np.zeros((3, nv))
            mujoco.mj_jacDot(m, d, jpd, jrd, p, self.feet[k])
            J = np.vstack([self.jp, self.jr])
            b = -np.vstack([jpd, jrd]) @ d.qvel - 20.0 * (J @ d.qvel)
            b[3:] += 60.0 * np.cross(d.xmat[self.feet[k]].reshape(3, 3)[:, 2], [0, 0, 1.0])
            task(J, b, 1000.0)
        nc = len(cpts)
        if swing is not None:
            k, pt = swing
            p = d.site_xpos[self.sole_sites[k]]
            mujoco.mj_jac(m, d, self.jp, self.jr, p, self.feet[k])
            task(self.jp.copy(), 400.0 * (pt - p) - 40.0 * (self.jp @ d.qvel), 100.0)
            R = d.xmat[self.feet[k]].reshape(3, 3)
            task(self.jr.copy(), 300.0 * (np.cross(R[:, 2], [0, 0, 1.0]) + np.cross(R[:, 0], [1.0, 0, 0])) - 30.0 * (self.jr @ d.qvel), 100.0)
        # COM (x, y)
        mujoco.mj_jacSubtreeCom(m, d, self.jc, self.torso)
        com = d.subtree_com[self.torso]
        cv = self.jc @ d.qvel
        a_com = 40.0 * (com_t - com[:2]) - 12.6 * cv[:2]
        task(self.jc[:2], a_com, 100.0)
        # torso height and upright torso
        mujoco.mj_jacBody(m, d, self.jp, self.jr, self.torso)
        task(self.jp[2:3].copy(), np.array([100.0 * (torso_z - d.xpos[self.torso, 2]) - 20.0 * (self.jp[2] @ d.qvel)]), 100.0)
        R = d.xmat[self.torso].reshape(3, 3)
        task(self.jr.copy(), 100.0 * (np.cross(R[:, 2], [0, 0, 1.0]) + np.cross(R[:, 0], [1.0, 0, 0])) - 20.0 * (self.jr @ d.qvel), 50.0)
        # shoulder + weak posture
        for name, q0, w in (("shoulder", shoulder, 10.0), ("hip_roll_l", 0, 0.01), ("hip_roll_r", 0, 0.01),
                            ("ankle_roll_l", 0, 0.01), ("ankle_roll_r", 0, 0.01), ("ankle_l", 0, 0.01), ("ankle_r", 0, 0.01)):
            j = m.joint(name).id
            e = np.zeros(nv)
            e[m.jnt_dofadr[j]] = 1.0
            task(e, np.array([60.0 * (q0 - d.qpos[m.jnt_qposadr[j]]) - 15.0 * d.qvel[m.jnt_dofadr[j]]]), w)

        J = np.vstack(rows_J)
        b = np.concatenate(rows_b)
        W = np.concatenate(rows_w)
        nx = nv + nu + 3 * nc
        P = np.zeros((nx, nx))
        q = np.zeros(nx)
        P[:nv, :nv] = 2 * (J.T * W) @ J + 1e-6 * np.eye(nv)
        q[:nv] = -2 * (J.T * W) @ b
        P[nv:nv + nu, nv:nv + nu] = 2e-4 * np.eye(nu)
        if nc:
            P[nv + nu:, nv + nu:] = 2e-6 * np.eye(3 * nc)

        # dynamics: M qacc - S^T tau - Jc^T f = -c
        Aeq = np.zeros((nv, nx))
        Aeq[:, :nv] = self.M
        Aeq[:, nv:nv + nu] = -self.S.T
        for i, (p, k) in enumerate(zip(cpts, cfoot)):
            mujoco.mj_jac(m, d, self.jp, self.jr, p, self.feet[k])
            Aeq[:, nv + nu + 3 * i: nv + nu + 3 * i + 3] = -self.jp.T
        beq = -c
        # inequalities
        Ain, lo, hi = [], [], []
        T = np.zeros((nu, nx))
        T[:, nv:nv + nu] = np.eye(nu)
        Ain.append(T)
        lo.append(-self.lim)
        hi.append(self.lim)
        # the planner is shifting weight off the foot it will lift: that foot's total normal force is
        # capped by (1 - u) * body weight, u = shift progress 0..1 (a soft penalty was not enough: the QP
        # kept ~22 % of the weight on it). An infeasible cap is reported by the solver, not hidden.
        unload_cap = None
        if getattr(self, "unload", None) is not None:
            kf, u = self.unload
            idx = [i for i, kk in enumerate(cfoot) if kk == kf]
            if idx:
                row = np.zeros((1, nx))
                for i in idx:
                    row[0, nv + nu + 3 * i + 2] = 1.0
                unload_cap = (1.0 - u) * self.weight
                Ain.append(row)
                lo.append(np.array([0.0]))
                hi.append(np.array([unload_cap]))
        for i in range(nc):
            base = nv + nu + 3 * i
            F = np.zeros((5, nx))
            F[0, base + 2] = 1.0                                   # 0 <= fz <= 2000
            F[1, base + 0], F[1, base + 2] = 1.0, -MU              # fx - mu fz <= 0
            F[2, base + 0], F[2, base + 2] = -1.0, -MU             # -fx - mu fz <= 0
            F[3, base + 1], F[3, base + 2] = 1.0, -MU
            F[4, base + 1], F[4, base + 2] = -1.0, -MU
            Ain.append(F)
            lo.append(np.array([0.0, -np.inf, -np.inf, -np.inf, -np.inf]))
            hi.append(np.array([2000.0, 0.0, 0.0, 0.0, 0.0]))
        A = np.vstack([Aeq] + Ain)
        l = np.concatenate([beq] + lo)
        u = np.concatenate([beq] + hi)
        solver = osqp.OSQP()
        solver.setup(sp.csc_matrix(np.triu(P)), q, sp.csc_matrix(A), l, u, verbose=False,
                     eps_abs=1e-5, eps_rel=1e-5, max_iter=8000, polishing=True)
        res = solver.solve()
        self.qp_stats["solves"] += 1
        t = float(d.time)
        if res.info.status not in ("solved", "solved inaccurate"):
            self.qp_stats["not_solved"] += 1
            self._event("qp_not_solved", t, status=res.info.status)
            d.ctrl[self.act] = self.tau  # hold the last command
            return
        x = res.x
        qacc, tau, f = x[:nv], x[nv:nv + nu], x[nv + nu:].reshape(-1, 3) if nc else np.zeros((0, 3))
        self.tau = np.clip(tau, -self.lim, self.lim)
        d.ctrl[self.act] = self.tau

        # ---------------- diagnostics
        tau_active = [self.m.actuator(int(a)).name for a, tv, lv in zip(self.act, tau, self.lim) if abs(tv) >= lv - 1e-3]
        fric_active = int(sum(1 for fi in f if fi[2] > 1.0 and max(abs(fi[0]), abs(fi[1])) >= MU * fi[2] - 1e-3))
        zero_corners = int(sum(1 for fi in f if fi[2] < 1.0))
        resid = W * (J @ qacc - b)
        com_resid = float(np.linalg.norm((self.jc[:2] @ qacc) - a_com))
        if tau_active:
            self.qp_stats["tau_active_steps"] += 1
            for a in tau_active:
                self._event(f"qp_tau_limit_{a}", t)
        if fric_active:
            self.qp_stats["friction_active_steps"] += 1
            self._event("qp_friction_cone_active", t)
        single = sum(stance) == 1
        if zero_corners and nc:
            self.qp_stats["edge_steps"] += 1
            if single:
                self._event("qp_cop_on_sole_edge_single_support", t, zero_corners=zero_corners)
        req_zmp = com[:2] - com[2] / G * a_com
        req_margin = _hull_margin(req_zmp, [p[:2] for p in cpts]) if nc >= 3 else -np.inf
        if nc >= 3 and req_margin < 0:
            self._event("required_cop_outside_support", t, margin_cm=round(req_margin * 100, 2), single_support=single)
        if nc and f[:, 2].sum() > 1:
            cop = (np.array([p[:2] for p in cpts]) * f[:, 2:3]).sum(0) / f[:, 2].sum()
        else:
            cop = np.full(2, np.nan)
        if self.k % 5 == 0:
            self.qp_rows.append(dict(t=round(t, 3), nc=nc, tau_active=tau_active, friction_active=fric_active,
                                     zero_force_corners=zero_corners, com_task_residual=round(com_resid, 3),
                                     required_cop=np.round(req_zmp, 4).tolist(), required_cop_margin_cm=round(req_margin * 100, 2) if np.isfinite(req_margin) else None,
                                     qp_cop=np.round(cop, 4).tolist(), weighted_residual=round(float(np.linalg.norm(resid)), 2)))
        self.k += 1

    def qp_summary(self):
        return dict(events=self.qp_events, stats=self.qp_stats, series_tail=self.qp_rows[-200:])
