"""Xandra-Lite skill controller. One ActionFrame interface for a human and an agent.

ActionFrame (the whole input; no coordinates, no teleport, no privileged commands):
  move   float in [-1, 1]   walk forward (+) / stop (0)          [backward walking: not in v0]
  crouch bool               lower the body by shortening the legs (physically, via leg motors)
  jump   bool               rising edge starts a jump; forward if move > 0, else vertical
  grab   bool               held: raise the arms; a hand that TOUCHES a grabbable geom is fixed to it
                            at the touch point (no pull); release detaches; flight follows physics

Skill layer -> a handful of task targets (COM, body height, torso orientation, swing-foot path,
shoulder angle) -> whole-body inverse dynamics with the real MuJoCo contacts -> 9 motor commands,
scaled into their limits. The only direct state writes are the seed at reset (logged).
"""
from __future__ import annotations

from dataclasses import dataclass, field

import mujoco
import numpy as np

from .model import ACTUATORS, limit_key

STAND_DROP = 0.03     # legs kept this much shorter than full length (away from the slide limit)
CROUCH_DROP = 0.36
STEP_LEN = 0.25
STEP_HEIGHT = 0.07
SHIFT_S, SWING_S = 0.7, 0.7
SHOULDER_UP = 3.05    # arms overhead
CONTACT_DIST = 0.003  # a geom within this distance of another counts as touching
COM_FWD = -0.02       # standing COM target relative to the sole centre (= 4 cm ahead of the ankle)


@dataclass
class ActionFrame:
    move: float = 0.0
    crouch: bool = False
    jump: bool = False
    grab: bool = False


@dataclass
class Log:
    """Everything that acts on the body, summarized; per-step arrays are not kept."""
    qpos_writes: list = field(default_factory=list)  # (time, reason)
    xfrc_events: list = field(default_factory=list)  # (t0, t1, body, wrench) declared test disturbances
    ctrl_abs_max: np.ndarray = field(default_factory=lambda: np.zeros(len(ACTUATORS)))
    ctrl_saturated_steps: np.ndarray = field(default_factory=lambda: np.zeros(len(ACTUATORS), int))
    contact_force_max: float = 0.0
    grip_force_max: float = 0.0
    grip_events: list = field(default_factory=list)  # (time, hand, 'attach'|'release', geom)
    task_scale_min: float = 1.0
    skill_events: list = field(default_factory=list)  # (time, from, to)
    steps: int = 0

    def summary(self):
        return dict(qpos_writes=self.qpos_writes, xfrc_events=self.xfrc_events,
                    ctrl_abs_max={a: round(float(v), 1) for a, v in zip(ACTUATORS, self.ctrl_abs_max)},
                    ctrl_saturated_fraction={a: round(float(v) / max(1, self.steps), 4) for a, v in zip(ACTUATORS, self.ctrl_saturated_steps)},
                    contact_force_max_N=round(self.contact_force_max, 1), grip_force_max_N=round(self.grip_force_max, 1),
                    grip_events=self.grip_events, task_scale_min=round(self.task_scale_min, 3),
                    skill_events=self.skill_events, sim_steps=self.steps)


def _smooth(u):
    u = min(1.0, max(0.0, u))
    return u * u * (3 - 2 * u)


class LiteController:
    def __init__(self, m: mujoco.MjModel, grabbable=()):
        self.m = m
        self.nv = m.nv
        self.torso = m.body("torso").id
        self.arms = m.body("arms").id
        self.feet = [m.body("foot_l").id, m.body("foot_r").id]
        self.soles = [m.geom("sole_l").id, m.geom("sole_r").id]
        self.sole_sites = [m.site("sole_l").id, m.site("sole_r").id]
        self.hands = [m.geom("hand_l").id, m.geom("hand_r").id]
        self.hand_sites = [m.site("hand_l").id, m.site("hand_r").id]
        self.act = np.array([m.actuator(a).id for a in ACTUATORS])
        self.act_dof = np.array([m.jnt_dofadr[m.actuator_trnid[i, 0]] for i in self.act])
        self.lo, self.hi = m.actuator_ctrlrange[self.act, 0], m.actuator_ctrlrange[self.act, 1]
        self.dof = {m.joint(j).name: m.jnt_dofadr[j] for j in range(m.njnt)}
        self.qadr = {m.joint(j).name: m.jnt_qposadr[j] for j in range(m.njnt)}
        self.grabbable = {m.geom(g).id for g in grabbable}
        self.grip_eq = [m.equality(n).id if mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_EQUALITY, n) >= 0 else -1
                        for n in ("grip_l", "grip_r")]
        self.M = np.zeros((self.nv, self.nv))
        self.jp, self.jr = np.zeros((3, self.nv)), np.zeros((3, self.nv))
        self.jpd, self.jrd = np.zeros((3, self.nv)), np.zeros((3, self.nv))
        self.log = Log()
        self.state = "stand"
        self.diag = {}

    # ---------------- reset (the only direct state write) ----------------
    def reset(self, d, joint_noise=0.0, seed=0, reason="reset"):
        m = self.m
        mujoco.mj_resetData(m, d)
        if joint_noise:
            rng = np.random.default_rng(seed)
            for n in ("hip_roll_l", "hip_pitch_l", "ankle_l", "ankle_roll_l", "hip_roll_r", "hip_pitch_r", "ankle_r", "ankle_roll_r", "shoulder"):
                d.qpos[self.qadr[n]] += rng.normal(0, joint_noise)
            for n in ("leg_l", "leg_r"):
                d.qpos[self.qadr[n]] = abs(rng.normal(0.03, joint_noise / 2))
        # drop the free body straight down until a sole touches the ground (no penetration)
        mujoco.mj_forward(m, d)
        low = self._lowest_sole_gap(d)
        d.qpos[2] -= low
        d.qvel[:] = 0
        mujoco.mj_forward(m, d)
        self.log.qpos_writes.append((round(float(d.time), 3), reason))
        self.state, self.t_state = "stand", 0.0
        self.prev = ActionFrame()
        self.stand_z = float(d.subtree_com[self.torso, 2]) - STAND_DROP
        self.ground_z = self._support_height(d)
        self.com_xy_target = None
        self.walk = None
        self.jump = None
        self.attached = [False, False]
        for e in self.grip_eq:
            if e >= 0:
                d.eq_active[e] = 0

    def _lowest_sole_gap(self, d):
        """Signed vertical gap between the lowest sole corner and whatever is under it (MuJoCo distance)."""
        best = np.inf
        for g in self.soles:
            for other in range(self.m.ngeom):
                if self.m.geom_bodyid[other] != 0 and self.m.body_rootid[self.m.geom_bodyid[other]] == self.torso:
                    continue
                dist = mujoco.mj_geomDistance(self.m, d, g, other, 1.0, None)
                best = min(best, dist)
        return 0.0 if not np.isfinite(best) else best

    def _support_height(self, d):
        return float(np.mean([self._sole_point(d, k)[2] for k in range(2)]))

    # ---------------- kinematics helpers ----------------
    def _sole_point(self, d, k):
        return d.site_xpos[self.sole_sites[k]].copy()

    def _contacts_of(self, d, geom):
        out = []
        for i in range(d.ncon):
            c = d.contact[i]
            if c.dist > CONTACT_DIST:
                continue
            if c.geom1 == geom or c.geom2 == geom:
                other = c.geom2 if c.geom1 == geom else c.geom1
                n = c.frame[:3].copy()
                if c.geom2 == geom:  # normal points from geom1 to geom2: make it point INTO our geom
                    pass
                else:
                    n = -n
                out.append((c.pos.copy(), n, other))
        return out

    def foot_contacts(self, d, k):
        return self._contacts_of(d, self.soles[k])

    def support_velocity(self, d, k, point):
        """6D velocity (lin, ang; world) of the body foot k stands on, at `point` (0 for static ground)."""
        cs = self.foot_contacts(d, k)
        if not cs:
            return np.zeros(6)
        b = self.m.geom_bodyid[cs[0][2]]
        if b == 0:
            return np.zeros(6)
        v = np.zeros(6)
        mujoco.mj_objectVelocity(self.m, d, mujoco.mjtObj.mjOBJ_BODY, b, v, 0)  # [ang, lin] at body origin
        ang, lin = v[:3], v[3:]
        return np.r_[lin + np.cross(ang, point - d.xpos[b]), ang]

    def hand_touching(self, d, k):
        for _p, _n, other in self._contacts_of(d, self.hands[k]):
            if other in self.grabbable:
                return other
        return None

    # ---------------- grip (physical touch only) ----------------
    def _attach(self, d, k, bar_geom):
        e = self.grip_eq[k]
        if e < 0:
            return
        m = self.m
        bar_body = m.geom_bodyid[bar_geom]
        hand = d.site_xpos[self.hand_sites[k]]
        # anchor on the bar = where the hand is now, so the constraint starts at zero error (no pull)
        rb = d.xmat[bar_body].reshape(3, 3)
        m.eq_data[e, 0:3] = m.site_pos[self.hand_sites[k]]
        m.eq_data[e, 3:6] = rb.T @ (hand - d.xpos[bar_body])
        m.eq_obj2id[e] = bar_body
        d.eq_active[e] = 1
        self.attached[k] = True
        self.log.grip_events.append((round(float(d.time), 3), "lr"[k], "attach", m.geom(bar_geom).name))

    def _release(self, d, k):
        e = self.grip_eq[k]
        if e >= 0 and d.eq_active[e]:
            d.eq_active[e] = 0
            self.log.grip_events.append((round(float(d.time), 3), "lr"[k], "release", ""))
        self.attached[k] = False

    def grip_force(self, d):
        tot = 0.0
        for e in self.grip_eq:
            if e < 0:
                continue
            for i in range(d.nefc):
                if d.efc_type[i] == mujoco.mjtConstraint.mjCNSTR_EQUALITY and d.efc_id[i] == e:
                    tot += d.efc_force[i] ** 2
        return float(np.sqrt(tot))

    # ---------------- skill state machine ----------------
    def _set_state(self, d, s):
        if s != self.state:
            self.log.skill_events.append((round(float(d.time), 3), self.state, s))
            self.state, self.t_state = s, 0.0

    def step(self, d, a: ActionFrame):
        """One control tick: ActionFrame -> d.ctrl. Call before mj_step."""
        self.t_state += self.m.opt.timestep
        jump_edge = a.jump and not self.prev.jump
        on = [len(self.foot_contacts(d, k)) > 0 for k in range(2)]
        # grip: only a physical touch while grab is held attaches; releasing grab detaches
        for k in range(2):
            if a.grab and not self.attached[k]:
                g = self.hand_touching(d, k)
                if g is not None:
                    self._attach(d, k, g)
            if not a.grab and self.attached[k]:
                self._release(d, k)
        hanging = any(self.attached)

        if hanging and not any(on):
            self._set_state(d, "hang")
        elif self.state == "hang" and not hanging:
            self._set_state(d, "flight")
        elif self.state in ("stand", "walk") and jump_edge and all(on):
            self.jump = dict(forward=a.move > 0.1, x0=float(d.subtree_com[self.torso, 0]))
            self._set_state(d, "jump_prep")
        elif self.state == "jump_push" and not any(on):
            self._set_state(d, "flight")
        elif self.state == "flight" and any(on) and self.t_state > 0.05:
            self._set_state(d, "land")
        elif self.state == "land" and self.t_state > 0.8:
            self._set_state(d, "stand")
        elif self.state == "stand" and abs(a.move) > 0.1 and all(on):
            self.walk = dict(phase="shift", swing=0, t=0.0, stop=False)
            self._set_state(d, "walk")
        elif self.state == "walk" and self.walk.get("done"):
            self._set_state(d, "stand")

        tau = self._control(d, a, on)
        d.ctrl[self.act] = tau
        self._log_step(d, tau)
        self.prev = a
        return tau

    # ---------------- task generation + whole-body solve ----------------
    def _control(self, d, a, on):
        st = self.state
        if st == "hang":
            return self._hang(d, a)
        if st == "flight":
            return self._flight(d, a)
        height_drop = CROUCH_DROP if a.crouch else STAND_DROP
        shoulder_target = SHOULDER_UP if a.grab else 0.0
        stance = [on[0], on[1]]
        swing = None
        com_t = None
        com_v = np.zeros(2)
        z_extra, z_vel = 0.0, 0.0
        mid = 0.5 * (self._sole_point(d, 0)[:2] + self._sole_point(d, 1)[:2])
        foot_c = [self._sole_point(d, k)[:2] + np.array([COM_FWD, 0.0]) for k in range(2)]

        if st == "walk":
            w = self.walk
            w["t"] += self.m.opt.timestep
            s = w["swing"]
            sup = 1 - s
            if w["phase"] == "shift":
                if "from" not in w:
                    w["from"] = d.subtree_com[self.torso, :2].copy()
                u = _smooth(w["t"] / SHIFT_S)
                tgt = foot_c[sup] * np.array([1, 0.92]) + np.array([0, 0.08 * mid[1]])
                com_t = (1 - u) * w["from"] + u * tgt
                if w["t"] >= SHIFT_S:
                    # decide the next swing target: forward step, or a closing step when move stopped
                    stop = abs(a.move) <= 0.1
                    if stop and abs(foot_c[s][0] - foot_c[sup][0]) < 0.03:
                        w.update(phase="center", t=0.0, **{"from": d.subtree_com[self.torso, :2].copy()})
                    else:
                        x_goal = foot_c[sup][0] + (0.0 if stop else STEP_LEN / 2 + STEP_LEN / 2)
                        if w.get("first", True):
                            x_goal = foot_c[sup][0] + (0.0 if stop else STEP_LEN / 2)
                        w.update(phase="swing", t=0.0, first=False, p0=self._sole_point(d, s).copy(),
                                 goal=np.array([x_goal - COM_FWD, self._sole_point(d, s)[1], self._sole_point(d, s)[2]]))
                        w["hold"] = com_t.copy()
            elif w["phase"] == "swing":
                stance[s] = False
                u = min(1.0, w["t"] / SWING_S)
                p = w["p0"] + _smooth(u) * (w["goal"] - w["p0"])
                p[2] = w["p0"][2] + STEP_HEIGHT * np.sin(np.pi * u) + (w["goal"][2] - w["p0"][2]) * _smooth(u)
                swing = (s, p)
                com_t = w["hold"]
                if u >= 1.0 and (on[s] or w["t"] > SWING_S + 0.3):
                    stance[s] = on[s]
                    w.update(phase="shift", swing=sup, t=0.0, **{"from": d.subtree_com[self.torso, :2].copy()})
            elif w["phase"] == "center":
                u = _smooth(w["t"] / SHIFT_S)
                com_t = (1 - u) * w["from"] + u * (mid + np.array([COM_FWD, 0.0]))
                if w["t"] >= SHIFT_S + 0.3:
                    w["done"] = True
        elif st == "jump_prep":
            u = _smooth(self.t_state / 0.45)
            z_extra = -0.16 * u
            back = -0.03 if self.jump["forward"] else 0.0
            com_t = mid + np.array([COM_FWD + back * u, 0.0])
            shoulder_target = SHOULDER_UP if a.grab else -0.5 * u
            if self.t_state >= 0.5:
                self._set_state(d, "jump_push")
        elif st == "jump_push":
            com_t = None  # explosive: acceleration targets below
        elif st == "land":
            z_extra = -0.12 * (1 - _smooth(self.t_state / 0.8))
            com_t = mid + np.array([COM_FWD, 0.0])
        else:  # stand
            com_t = mid + np.array([COM_FWD, 0.0])

        # a foot that should support (stand/land/walk-shift) but does not touch anything is lowered
        # straight down to the other sole's height ("seek ground"); its leg gets no other job
        if swing is None and st in ("stand", "land", "walk", "jump_prep") and sum(stance) == 1:
            k = 0 if not stance[0] else 1
            p = self._sole_point(d, k)
            swing = (k, np.array([p[0], p[1], self._sole_point(d, 1 - k)[2]]))
        com_z_t = self.ground_z_now(d, stance) + (self.stand_z - self.ground_z) - (height_drop - STAND_DROP) + z_extra
        return self._wbc(d, stance, com_t, com_z_t, swing, shoulder_target, push=(st == "jump_push"))

    def ground_z_now(self, d, stance):
        pts = [self._sole_point(d, k)[2] for k in range(2) if stance[k]] or [self._sole_point(d, k)[2] for k in range(2)]
        return float(np.mean(pts))

    def _wbc(self, d, stance, com_t, com_z_t, swing, shoulder_target, push=False):
        m, nv = self.m, self.nv
        rows, rhs, wts = [], [], []

        def add(J, b, w):
            rows.append(J)
            rhs.append(b)
            wts.append(np.full(len(b), w))

        # stance feet stay put
        for k in range(2):
            if stance[k]:
                p = self._sole_point(d, k)
                mujoco.mj_jac(m, d, self.jp, self.jr, p, self.feet[k])
                mujoco.mj_jacDot(m, d, self.jpd, self.jrd, p, self.feet[k])
                J = np.vstack([self.jp, self.jr])
                Jd = np.vstack([self.jpd, self.jrd])
                b = -Jd @ d.qvel - 20.0 * (J @ d.qvel - self.support_velocity(d, k, p))
                # a stance sole on an edge is rolled flat onto the ground (normal from the contacts)
                cs = self.foot_contacts(d, k)
                up = np.mean([c[1] for c in cs], axis=0) if cs else np.array([0, 0, 1.0])
                b[3:] += 60.0 * np.cross(d.xmat[self.feet[k]].reshape(3, 3)[:, 2], up / np.linalg.norm(up))
                add(J, b, 100.0)
        # swing foot path, sole kept level
        if swing is not None:
            k, p_t = swing
            p = self._sole_point(d, k)
            mujoco.mj_jac(m, d, self.jp, self.jr, p, self.feet[k])
            v = self.jp @ d.qvel
            add(self.jp.copy(), 400.0 * (p_t - p) - 40.0 * v, 30.0)
            R = d.xmat[self.feet[k]].reshape(3, 3)
            err = np.cross(R[:, 2], [0, 0, 1.0]) + 0.5 * np.cross(R[:, 0], [1, 0, 0.0])
            add(self.jr.copy(), 300.0 * err - 30.0 * (self.jr @ d.qvel), 30.0)
        # COM
        Jc = np.zeros((3, nv))
        mujoco.mj_jacSubtreeCom(m, d, Jc, self.torso)
        c, cv = d.subtree_com[self.torso], Jc @ d.qvel
        vs = [self.support_velocity(d, k, self._sole_point(d, k))[:3] for k in range(2) if stance[k]]
        cv = cv - (np.mean(vs, axis=0) if vs else 0.0)  # COM velocity relative to the support
        if push:
            fwd = 4.0 if self.jump["forward"] else 0.0
            add(Jc, np.array([fwd, 0.0, 30.0]) - np.array([0, 4.0 * cv[1], 0]), 10.0)
        else:
            a_xy = 25.0 * (com_t - c[:2]) - 10.0 * cv[:2] if com_t is not None else -10.0 * cv[:2]
            add(Jc, np.r_[a_xy, 60.0 * (com_z_t - c[2]) - 15.0 * cv[2]], 10.0)
        # torso upright, heading +X
        mujoco.mj_jacBody(m, d, self.jp, self.jr, self.torso)
        R = d.xmat[self.torso].reshape(3, 3)
        err = np.cross(R[:, 2], [0, 0, 1.0]) + np.cross(R[:, 0], [1, 0, 0.0])
        add(self.jr.copy(), 80.0 * err - 18.0 * (self.jr @ d.qvel), 10.0)
        # shoulder
        e = np.zeros(nv)
        e[self.dof["shoulder"]] = 1
        add(e[None], np.array([60.0 * (shoulder_target - d.qpos[self.qadr["shoulder"]]) - 15.0 * d.qvel[self.dof["shoulder"]]]), 5.0)
        # weak posture regularization (keeps the null space quiet)
        for n, q0 in (("hip_roll_l", 0), ("hip_roll_r", 0), ("ankle_l", 0), ("ankle_r", 0), ("ankle_roll_l", 0), ("ankle_roll_r", 0)):
            e = np.zeros(nv)
            e[self.dof[n]] = 1
            add(e[None], np.array([30.0 * (q0 - d.qpos[self.qadr[n]]) - 8.0 * d.qvel[self.dof[n]]]), 0.05)
        A = np.vstack(rows)
        b = np.concatenate(rhs)
        W = np.concatenate(wts)
        Aw = A * W[:, None]
        qacc = np.linalg.solve(A.T @ Aw + 1e-6 * np.eye(nv), Aw.T @ b)

        mujoco.mj_fullM(m, d, self.M)
        f1 = self.M @ qacc + d.qfrc_bias
        f0 = d.qfrc_bias.copy()
        G, normal = self._contact_basis(d, stance)
        t1 = self._torques(f1, G, normal)
        t0 = self._torques(f0, G, normal)
        alpha, dtau = 1.0, t1 - t0
        with np.errstate(divide="ignore", invalid="ignore"):
            for lim in (self.hi, self.lo):
                r = (lim - t0) / dtau
                ok = np.isfinite(r) & (r > 0) & (np.sign(dtau) == np.sign(lim - t0))
                if ok.any():
                    alpha = min(alpha, float(r[ok].min()))
        alpha = max(0.0, alpha)
        tau = t1 if alpha >= 1.0 else self._torques(alpha * (self.M @ qacc) + f0, G, normal)
        self.diag = dict(alpha=alpha, stance=list(stance))
        self.log.task_scale_min = min(self.log.task_scale_min, alpha)
        return np.clip(tau, self.lo, self.hi)

    def _contact_basis(self, d, stance):
        """Generalized-force columns of every real contact point of the stance soles."""
        cols, normal = [], []
        for k in range(2):
            if not stance[k]:
                continue
            cs = self.foot_contacts(d, k)
            for p, n, _o in cs:
                mujoco.mj_jac(self.m, d, self.jp, self.jr, p, self.feet[k])
                cols.append(self.jp.T @ n)
                normal.append(True)
            if cs:
                p = np.mean([c[0] for c in cs], axis=0)
                mujoco.mj_jac(self.m, d, self.jp, self.jr, p, self.feet[k])
                for v in (np.array([1.0, 0, 0]), np.array([0, 1.0, 0])):
                    cols.append(self.jp.T @ v)
                    normal.append(False)
                cols.append(self.jr.T @ np.array([0, 0, 1.0]))
                normal.append(False)
        return (np.array(cols).T if cols else None), np.array(normal, bool)

    def _torques(self, f, G, normal):
        f = f.copy()
        if G is not None:
            a, b = G[:6], f[:6]
            free = np.ones(G.shape[1], bool)
            lam = np.zeros(G.shape[1])
            for _ in range(15):  # normal forces >= 0 (active set)
                af = a[:, free]
                lam = np.zeros(G.shape[1])
                lam[free] = np.linalg.solve(af.T @ af + 1e-6 * np.eye(af.shape[1]), af.T @ b)
                neg = free & normal & (lam < 0)
                if not neg.any():
                    break
                free &= ~neg
            f = f - G @ lam
        return f[self.act_dof]

    # ---------------- no ground support ----------------
    def _pd(self, d, targets, gains):
        tau = np.zeros(len(self.act))
        for i, a in enumerate(ACTUATORS):
            q, v = d.qpos[self.qadr[a]], d.qvel[self.dof[a]]
            kp, kd = gains[limit_key(a)]
            tau[i] = kp * (targets.get(a, 0.0) - q) - kd * v
        return np.clip(tau, self.lo, self.hi)

    def _flight(self, d, a):
        fwd = 0.45 if (self.jump or {}).get("forward") else 0.15
        t = dict(hip_pitch_l=fwd, hip_pitch_r=fwd, leg_l=0.10, leg_r=0.10,
                 ankle_l=0.0, ankle_r=0.0, shoulder=SHOULDER_UP if a.grab else d.qpos[self.qadr["shoulder"]])
        return self._pd(d, t, dict(hip_roll=(300, 20), hip_pitch=(300, 20), leg=(3000, 150), ankle=(60, 3), ankle_roll=(60, 3), shoulder=(200, 15)))

    def _hang(self, d, a):
        # pump: legs forward while the body swings forward (move > 0); otherwise hang quietly
        R = d.xmat[self.torso].reshape(3, 3)
        swing_rate = -(d.qvel[3:6] @ np.array([0, 1.0, 0]))  # pitch rate about +Y (local ~ world here)
        pump = 0.0
        if abs(a.move) > 0.1:
            pump = 0.8 * np.tanh(3.0 * swing_rate) * np.sign(a.move)
        t = dict(hip_pitch_l=0.2 + pump, hip_pitch_r=0.2 + pump, leg_l=0.15, leg_r=0.15,
                 shoulder=d.qpos[self.qadr["shoulder"]])
        tau = self._pd(d, t, dict(hip_roll=(300, 20), hip_pitch=(400, 30), leg=(3000, 150), ankle=(60, 3), ankle_roll=(60, 3), shoulder=(0, 2)))
        return tau

    # ---------------- logging ----------------
    def _log_step(self, d, tau):
        L = self.log
        L.steps += 1
        L.ctrl_abs_max = np.maximum(L.ctrl_abs_max, np.abs(tau))
        L.ctrl_saturated_steps += ((tau <= self.lo + 1e-9) | (tau >= self.hi - 1e-9)).astype(int)
        f6 = np.zeros(6)
        for i in range(d.ncon):
            mujoco.mj_contactForce(self.m, d, i, f6)
            L.contact_force_max = max(L.contact_force_max, abs(f6[0]))
        if any(self.attached):
            L.grip_force_max = max(L.grip_force_max, self.grip_force(d))
