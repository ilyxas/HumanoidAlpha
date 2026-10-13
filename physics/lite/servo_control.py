"""Xandra-Lite, path 2: position servos + kinematic planner (IK). Experimental, headless.

Input: ActionFrame (move, crouch, jump, grab) - the same for a human and an agent.
Skill layer keeps an explicit support state (double / single_l / single_r) and puts the COM inside
the current support polygon with a margin; a foot is lifted only after the MEASURED COM has
arrived over the other foot and the foot is unloaded. The planner turns a few targets (stance
feet where they actually are, swing foot path, COM, body height, upright torso, shoulder) into
joint targets by damped least-squares IK on a private MjData. The servos (force-limited,
see model.py) are the only actuators; MuJoCo computes what really happens.
A slow COM stabilizer shifts the planned COM against the measured COM error (logged).
Direct state writes: only the seed at reset (logged).
"""
from __future__ import annotations

from dataclasses import dataclass, field

import mujoco
import numpy as np

from .model import ACTUATORS, HIP_Z, JOINT_RANGES, LIMITS, limit_key

STAND_DROP = 0.03
CROUCH_DROP = 0.40
STEP_LEN = 0.20
STEP_HEIGHT = 0.06
SWING_S = 0.8
SHIFT_S = 1.0           # COM moves along a smooth path over this time (a step change flung the body)
SHIFT_TIMEOUT_S = 3.0
COM_FWD = -0.02          # COM target relative to a sole centre (= 4 cm ahead of the ankle)
LAT_MARGIN = 0.020       # single support: COM this far inside from the sole centre towards the body
SETTLE_VEL = 0.03        # ... and moving slower than this [m/s] (an outward drift at lift-off tipped her over the sole edge)
ARRIVE_TOL = 0.012       # measured COM must be this close to its target before a foot lifts
UNLOAD_FRAC = 0.12       # ... and the swing foot may carry at most this fraction of body weight
CONTACT_DIST = 0.003
STAB_GAIN = 0.0          # integral COM stabilizer: 1.5/s made the stand oscillate and fall (measured) -> off
STAB_KP, STAB_KD = 0.8, 0.25   # PD COM stabilizer (error, velocity [s]): plans against overshoot
STAB_MAX = 0.04


@dataclass
class ActionFrame:
    move: float = 0.0
    crouch: bool = False
    jump: bool = False
    grab: bool = False


@dataclass
class Log:
    qpos_writes: list = field(default_factory=list)
    xfrc_events: list = field(default_factory=list)
    force_abs_max: dict = field(default_factory=dict)
    saturated_steps: dict = field(default_factory=dict)
    positive_work_J: float = 0.0
    peak_power_W: float = 0.0
    contact_force_max_N: float = 0.0
    stance_slip_max_cm: float = 0.0
    stabilizer_max_cm: float = 0.0
    support_events: list = field(default_factory=list)
    skill_events: list = field(default_factory=list)
    steps: int = 0

    def summary(self):
        return dict(qpos_writes=self.qpos_writes, xfrc_events=self.xfrc_events,
                    force_abs_max={k: round(v, 1) for k, v in self.force_abs_max.items()},
                    saturated_fraction={k: round(v / max(1, self.steps), 4) for k, v in self.saturated_steps.items()},
                    positive_work_J=round(self.positive_work_J, 1), peak_power_W=round(self.peak_power_W, 1),
                    contact_force_max_N=round(self.contact_force_max_N, 1), stance_slip_max_cm=round(self.stance_slip_max_cm, 2),
                    stabilizer_max_cm=round(self.stabilizer_max_cm, 2), support_events=self.support_events[-40:],
                    skill_events=self.skill_events[-40:], sim_steps=self.steps)


def smooth(u):
    u = min(1.0, max(0.0, u))
    return u * u * (3 - 2 * u)


class ServoController:
    def __init__(self, m: mujoco.MjModel, plan_every: int = 5):
        self.m = m
        self.pd = mujoco.MjData(m)  # planner's private kinematic copy
        self.plan_every = plan_every
        self.torso = m.body("torso").id
        self.feet = [m.body("foot_l").id, m.body("foot_r").id]
        self.soles = [m.geom("sole_l").id, m.geom("sole_r").id]
        self.sole_sites = [m.site("sole_l").id, m.site("sole_r").id]
        self.act = np.array([m.actuator(a).id for a in ACTUATORS])
        self.act_q = np.array([m.jnt_qposadr[m.actuator_trnid[i, 0]] for i in self.act])
        self.act_v = np.array([m.jnt_dofadr[m.actuator_trnid[i, 0]] for i in self.act])
        self.lim = np.array([LIMITS[limit_key(a)] for a in ACTUATORS])
        self.qlo = np.array([JOINT_RANGES[limit_key(a)][0] for a in ACTUATORS])
        self.qhi = np.array([JOINT_RANGES[limit_key(a)][1] for a in ACTUATORS])
        self.shoulder_q = m.jnt_qposadr[m.joint("shoulder").id]
        self.weight = float(m.body_subtreemass[self.torso]) * 9.81
        self.jp, self.jr = np.zeros((3, m.nv)), np.zeros((3, m.nv))
        self.log = Log()
        self.diag = {}

    # ------------------------------------------------------------ reset
    def reset(self, d, joint_noise=0.0, seed=0):
        m = self.m
        mujoco.mj_resetData(m, d)
        if joint_noise:
            rng = np.random.default_rng(seed)
            for a, qa, lo, hi in zip(ACTUATORS, self.act_q, self.qlo, self.qhi):
                if a.startswith("leg"):
                    d.qpos[qa] = np.clip(abs(rng.normal(STAND_DROP, joint_noise / 2)), lo, hi)
                elif a != "shoulder":
                    d.qpos[qa] = np.clip(rng.normal(0, joint_noise), lo, hi)
        mujoco.mj_forward(m, d)
        d.qpos[2] -= self._gap_below_soles(d)  # lower the free body until a sole touches
        d.qvel[:] = 0
        mujoco.mj_forward(m, d)
        self.log.qpos_writes.append((round(float(d.time), 3), "reset seed"))
        self.plan_q = d.qpos.copy()
        d.ctrl[self.act] = d.qpos[self.act_q]
        self.support = "double" if all(self.on_ground(d)) else ("single_l" if self.on_ground(d)[0] else "single_r")
        self.skill, self.t_skill = "stand", 0.0
        self.walk = None
        self.stab = np.zeros(2)
        self.last_com_t = 0.5 * (d.site_xpos[self.sole_sites[0]][:2] + d.site_xpos[self.sole_sites[1]][:2]) + np.array([COM_FWD, 0.0])
        self.ground_z = float(np.mean([d.site_xpos[s][2] for s in self.sole_sites]))
        self.anchor = [d.site_xpos[s].copy() for s in self.sole_sites]
        self.k = 0
        self.prev = ActionFrame()

    def _gap_below_soles(self, d):
        best = np.inf
        own = {i for i in range(self.m.ngeom) if self.m.body_rootid[self.m.geom_bodyid[i]] == self.torso}
        for g in self.soles:
            for o in range(self.m.ngeom):
                if o not in own:
                    best = min(best, mujoco.mj_geomDistance(self.m, d, g, o, 2.0, None))
        return 0.0 if not np.isfinite(best) else best

    # ------------------------------------------------------------ sensing
    def foot_force(self, d, k):
        f6, tot = np.zeros(6), 0.0
        for i in range(d.ncon):
            c = d.contact[i]
            if (c.geom1 == self.soles[k] or c.geom2 == self.soles[k]) and c.dist < CONTACT_DIST:
                mujoco.mj_contactForce(self.m, d, i, f6)
                tot += f6[0]
        return tot

    def on_ground(self, d):
        out = [False, False]
        for i in range(d.ncon):
            c = d.contact[i]
            for k in range(2):
                if (c.geom1 == self.soles[k] or c.geom2 == self.soles[k]) and c.dist < CONTACT_DIST:
                    out[k] = True
        return out

    def sole(self, d, k):
        return d.site_xpos[self.sole_sites[k]].copy()

    def com(self, d):
        return d.subtree_com[self.torso].copy()

    # ------------------------------------------------------------ skills
    def _set_skill(self, d, s):
        if s != self.skill:
            self.log.skill_events.append((round(float(d.time), 3), self.skill, s))
            self.skill, self.t_skill = s, 0.0

    def _set_support(self, d, s):
        if s != self.support:
            self.log.support_events.append((round(float(d.time), 3), self.support, s))
            self.support = s

    def step(self, d, a: ActionFrame):
        dt = self.m.opt.timestep
        self.t_skill += dt
        on = self.on_ground(d)
        com = self.com(d)
        soles = [self.sole(d, k) for k in range(2)]
        height_drop = CROUCH_DROP if a.crouch else STAND_DROP
        swing = None

        if self.skill == "stand":
            com_t = 0.5 * (soles[0][:2] + soles[1][:2]) + np.array([COM_FWD, 0.0])
            if all(on):
                self._set_support(d, "double")
            if abs(a.move) > 0.1 and all(on):
                self.walk = dict(phase="shift", sw=0, t=0.0, first=True)
                self._set_skill(d, "walk")
        if self.skill == "walk":
            w = self.walk
            w["t"] += dt
            sw, st = w["sw"], 1 - w["sw"]
            stance_c = soles[st][:2] + np.array([COM_FWD, 0.0])
            inward = -np.sign(soles[st][1] - 0.5 * (soles[0][1] + soles[1][1]))
            over_stance = stance_c + np.array([0.0, inward * LAT_MARGIN])
            if w["phase"] == "shift":
                self._set_support(d, "double")
                if "from" not in w:
                    w["from"] = self.last_com_t.copy()
                com_t = w["from"] + smooth(w["t"] / SHIFT_S) * (over_stance - w["from"])
                jc = np.zeros((3, self.m.nv))
                mujoco.mj_jacSubtreeCom(self.m, d, jc, self.torso)
                arrived = np.linalg.norm(com[:2] - over_stance) < ARRIVE_TOL and np.linalg.norm((jc @ d.qvel)[:2]) < SETTLE_VEL
                unloaded = self.foot_force(d, sw) < UNLOAD_FRAC * self.weight
                stop = abs(a.move) <= 0.1
                aligned = abs(soles[sw][0] - soles[st][0]) < 0.02
                if stop and aligned:
                    w.update(phase="center", t=0.0, **{"from": com_t.copy()})
                elif arrived and unloaded and w["t"] >= SHIFT_S:
                    adv = 0.0 if stop else (STEP_LEN / 2 if w["first"] else STEP_LEN)
                    goal = np.array([soles[st][0] + adv, soles[sw][1], soles[st][2]])
                    w.update(phase="swing", t=0.0, first=False, p0=soles[sw].copy(), goal=goal, hold=com_t.copy())
                elif w["t"] > SHIFT_TIMEOUT_S:
                    self.log.skill_events.append((round(float(d.time), 3), "walk", "shift timeout"))
                    w.update(t=0.0)
            if w["phase"] == "swing":
                self._set_support(d, "single_l" if st == 0 else "single_r")
                u = min(1.0, w["t"] / SWING_S)
                p = w["p0"] + smooth(u) * (w["goal"] - w["p0"])
                p[2] = w["p0"][2] + STEP_HEIGHT * np.sin(np.pi * u) + smooth(u) * (w["goal"][2] - w["p0"][2])
                if u >= 1.0:
                    p[2] -= min(0.03, 0.05 * (w["t"] - SWING_S))  # keep reaching down until it touches
                swing = (sw, p)
                com_t = w["hold"]
                if u >= 1.0 and on[sw]:
                    w.update(phase="shift", sw=st, t=0.0, **{"from": com_t.copy()})
                    self._set_support(d, "double")
            if w["phase"] == "center":
                self._set_support(d, "double")
                mid_t = 0.5 * (soles[0][:2] + soles[1][:2]) + np.array([COM_FWD, 0.0])
                com_t = w["from"] + smooth(w["t"] / SHIFT_S) * (mid_t - w["from"])
                if np.linalg.norm(com[:2] - mid_t) < ARRIVE_TOL and w["t"] > SHIFT_S:
                    self._set_skill(d, "stand")

        self.last_com_t = com_t.copy()
        # slow COM stabilizer: plan the COM a little against the measured error (bounded)
        err = com_t - com[:2]
        jc = np.zeros((3, self.m.nv))
        mujoco.mj_jacSubtreeCom(self.m, d, jc, self.torso)
        com_vel = (jc @ d.qvel)[:2]
        self.stab = np.clip(STAB_KP * err - STAB_KD * com_vel, -STAB_MAX, STAB_MAX)
        self.log.stabilizer_max_cm = max(self.log.stabilizer_max_cm, float(np.linalg.norm(self.stab)) * 100)
        plan_com = com_t + self.stab

        stance = [on[k] and not (swing is not None and swing[0] == k) for k in range(2)]
        if self.k % self.plan_every == 0:
            torso_z = float(np.mean([soles[k][2] for k in range(2) if stance[k]] or [soles[0][2], soles[1][2]])) + HIP_Z - height_drop
            shoulder = 3.05 if a.grab else 0.0
            self._plan(d, stance, swing, plan_com, torso_z, shoulder)
        self.k += 1
        d.ctrl[self.act] = np.clip(self.plan_q[self.act_q], self.qlo, self.qhi)
        self._log(d, on)
        self.prev = a
        self.diag = dict(skill=self.skill, support=self.support, com_err_cm=round(float(np.linalg.norm(err)) * 100, 2))

    # ------------------------------------------------------------ planner (IK)
    def _plan(self, d, stance, swing, com_xy, torso_z, shoulder, iters=4):
        m, pd = self.m, self.pd
        # Feed-forward pose anchored on the stance feet: the planner's root is its own (warm start),
        # NOT the measured root - chasing the measured lean through IK made a growing oscillation.
        q = self.plan_q.copy()
        for _ in range(iters):
            pd.qpos[:] = q
            mujoco.mj_kinematics(m, pd)
            mujoco.mj_comPos(m, pd)
            rows, errs, wts = [], [], []

            def add(J, e, w):
                rows.append(J)
                errs.append(e)
                wts.append(np.full(len(e), w))

            for k in range(2):
                tgt = None
                if stance[k]:
                    tgt = d.site_xpos[self.sole_sites[k]].copy()  # stays where it really is
                elif swing is not None and swing[0] == k:
                    tgt = swing[1]
                if tgt is None:
                    continue
                p = pd.site_xpos[self.sole_sites[k]]
                mujoco.mj_jacSite(m, pd, self.jp, self.jr, self.sole_sites[k])
                add(self.jp.copy(), tgt - p, 50.0)
                R = pd.xmat[self.feet[k]].reshape(3, 3)
                add(self.jr.copy(), np.cross(R[:, 2], [0, 0, 1.0]) + np.cross(R[:, 0], [1.0, 0, 0]), 20.0)
            jc = np.zeros((3, m.nv))
            mujoco.mj_jacSubtreeCom(m, pd, jc, self.torso)
            add(jc[:2], com_xy - pd.subtree_com[self.torso, :2], 10.0)
            mujoco.mj_jacBody(m, pd, self.jp, self.jr, self.torso)
            add(self.jp[2:3].copy(), np.array([torso_z - pd.xpos[self.torso, 2]]), 10.0)
            R = pd.xmat[self.torso].reshape(3, 3)
            add(self.jr.copy(), np.cross(R[:, 2], [0, 0, 1.0]) + np.cross(R[:, 0], [1.0, 0, 0]), 10.0)
            e = np.zeros(m.nv)
            e[m.jnt_dofadr[m.joint("shoulder").id]] = 1
            add(e[None], np.array([shoulder - q[self.shoulder_q]]), 5.0)
            J = np.vstack(rows)
            err = np.concatenate(errs)
            W = np.concatenate(wts)
            Jw = J * W[:, None]
            dq = np.linalg.solve(J.T @ Jw + 1e-3 * np.eye(m.nv), Jw.T @ err)
            mujoco.mj_integratePos(m, q, dq, 1.0)
            q[self.act_q] = np.clip(q[self.act_q], self.qlo, self.qhi)
        self.plan_q = q

    # ------------------------------------------------------------ logging
    def _log(self, d, on):
        L = self.log
        L.steps += 1
        f = d.actuator_force[self.act]
        for a, fv, lim in zip(ACTUATORS, f, self.lim):
            L.force_abs_max[a] = max(L.force_abs_max.get(a, 0.0), abs(float(fv)))
            if abs(fv) >= lim - 1e-6:
                L.saturated_steps[a] = L.saturated_steps.get(a, 0) + 1
        p = f * d.actuator_velocity[self.act]
        L.positive_work_J += float(np.sum(np.clip(p, 0, None))) * self.m.opt.timestep
        L.peak_power_W = max(L.peak_power_W, float(np.sum(np.abs(p))))
        L.contact_force_max_N = max(L.contact_force_max_N, max(self.foot_force(d, 0), self.foot_force(d, 1)))
        for k in range(2):
            s = d.site_xpos[self.sole_sites[k]]
            if on[k] and self.foot_force(d, k) > 0.2 * self.weight:
                L.stance_slip_max_cm = max(L.stance_slip_max_cm, float(np.linalg.norm(s[:2] - self.anchor[k][:2])) * 100)
            else:
                self.anchor[k] = s.copy()
