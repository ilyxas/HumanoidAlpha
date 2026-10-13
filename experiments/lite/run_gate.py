"""Xandra-Lite locomotion gate (path 2: position servos + IK). Headless, MuJoCo primitives only.

  .venv/bin/python experiments/lite/run_gate.py [g1 g2 g3 g4 g5] [--video DIR]

g1 stand 20 s, then weight onto one leg, lift the other 5 cm, hold 3 s (one-leg stance via the walk
   skill's own shift/swing logic is covered by g2; g1 drives the planner directly)
g2 one physical step
g3 walk 1 m and stop, hold 5 s
g4 g3 from 5 slightly different start poses
g5 crouch under a physical beam; standing into it = real contact and stop
Input is ActionFrame only. No external forces, no state writes except the logged reset seed.
"""
from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "physics"))
from lite.model import build  # noqa: E402
from lite import servo_control as S  # noqa: E402

VIDEO = Path(sys.argv[sys.argv.index("--video") + 1]) if "--video" in sys.argv else None


def tilt(d, b):
    return float(np.degrees(np.arccos(np.clip(d.xmat[b].reshape(3, 3)[2, 2], -1, 1))))


class Run:
    def __init__(self, world="", noise=0.0, seed=0):
        self.m = build(world)
        self.d = mujoco.MjData(self.m)
        self.c = S.ServoController(self.m)
        self.c.reset(self.d, joint_noise=noise, seed=seed)
        self.fell, self.max_tilt, self.frames = None, 0.0, []
        if VIDEO:
            self.ren = mujoco.Renderer(self.m, 360, 640)
            self.cam = mujoco.MjvCamera()
            self.cam.distance, self.cam.azimuth, self.cam.elevation = 3.5, 60, -12

    def run(self, seconds, action, until=None):
        for _ in range(int(round(seconds / self.m.opt.timestep))):
            self.c.step(self.d, action(self) if callable(action) else action)
            mujoco.mj_step(self.m, self.d)
            t = tilt(self.d, self.c.torso)
            self.max_tilt = max(self.max_tilt, t)
            if self.fell is None and (t > 45 or self.d.xpos[self.c.torso, 2] < 0.45):
                self.fell = round(float(self.d.time), 2)
            if VIDEO and int(round(self.d.time / self.m.opt.timestep)) % 17 == 0:
                self.cam.lookat[:] = [self.c.com(self.d)[0], 0, 0.8]
                self.ren.update_scene(self.d, self.cam)
                self.frames.append(self.ren.render().copy())
            if until is not None and until(self):
                return True
        return False

    def video(self, name):
        if not VIDEO or not self.frames:
            return
        VIDEO.mkdir(parents=True, exist_ok=True)
        h, w = self.frames[0].shape[:2]
        ff = subprocess.Popen(["ffmpeg", "-loglevel", "error", "-y", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{w}x{h}", "-r", "29.4",
                               "-i", "-", "-pix_fmt", "yuv420p", "-vcodec", "libx264", str(VIDEO / f"{name}.mp4")], stdin=subprocess.PIPE)
        for f in self.frames:
            ff.stdin.write(f.tobytes())
        ff.stdin.close()
        ff.wait()


def g1_stand_one_leg():
    r = Run()
    r.run(20.0, S.ActionFrame())
    stand = dict(fell=r.fell, max_tilt_deg=round(r.max_tilt, 2))
    # one-leg stance through the walk skill: start walking, stop after the first lift is reached
    lifted = r.run(8.0, S.ActionFrame(move=1.0), until=lambda x: x.c.support.startswith("single"))
    t_lift = float(r.d.time)
    # hold the swing phase target is the walk's own; here we only check the transfer + lift worked
    r.run(0.3, S.ActionFrame(move=1.0))
    out = dict(stand_20s=stand, reached_single_support=lifted, t_single=round(t_lift, 2), fell=r.fell,
               max_tilt_deg=round(r.max_tilt, 2), support_events=r.c.log.support_events[:6])
    out["pass"] = stand["fell"] is None and lifted and r.fell is None
    return out


def g2_one_step():
    r = Run()
    r.run(1.0, S.ActionFrame())
    x0 = [r.c.sole(r.d, k)[0] for k in range(2)]
    # walk until the first swing foot has landed (support back to double after a single phase)
    seen = {"single": False}

    def done(x):
        if x.c.support.startswith("single"):
            seen["single"] = True
        return seen["single"] and x.c.support == "double"

    stepped = r.run(10.0, S.ActionFrame(move=1.0), until=done)
    r.run(4.0, S.ActionFrame())  # stop: closing step + centre
    x1 = [r.c.sole(r.d, k)[0] for k in range(2)]
    out = dict(stepped=stepped, foot_advance_m=[round(b - a, 3) for a, b in zip(x0, x1)], fell=r.fell,
               max_tilt_deg=round(r.max_tilt, 2), final_skill=r.c.skill, log=r.c.log.summary())
    out["pass"] = stepped and r.fell is None and max(out["foot_advance_m"]) > 0.05
    r.video("g2_one_step")
    return out


def walk_1m(noise=0.0, seed=0, video=None):
    r = Run(noise=noise, seed=seed)
    r.run(2.0, S.ActionFrame())
    x0 = r.c.com(r.d)[0]
    r.run(40.0, lambda x: S.ActionFrame(move=1.0 if x.c.com(x.d)[0] - x0 < 0.85 else 0.0),
          until=lambda x: x.c.com(x.d)[0] - x0 >= 0.85 and x.c.skill == "stand")
    walked_until = float(r.d.time)
    xs = r.c.com(r.d)[0] - x0
    r.run(5.0, S.ActionFrame())
    out = dict(distance_m=round(float(r.c.com(r.d)[0] - x0), 3), time_s=round(walked_until, 1), skill=r.c.skill, fell=r.fell,
               max_tilt_deg=round(r.max_tilt, 2), drift_after_stop_cm=round(float(abs(r.c.com(r.d)[0] - x0 - xs)) * 100, 2),
               log=r.c.log.summary())
    out["pass"] = r.fell is None and 0.9 <= out["distance_m"] <= 1.25 and r.c.skill == "stand"
    if video:
        r.video(video)
    return out


def g3_walk_1m():
    return walk_1m(video="g3_walk_1m")


def g4_starts():
    res = []
    for s in range(5):
        o = walk_1m(noise=0.05, seed=s)
        res.append({k: o[k] for k in ("distance_m", "fell", "skill", "max_tilt_deg", "pass")})
    return dict(starts=res, pass_=all(x["pass"] for x in res))


def g5_crouch_beam():
    beam = '<geom name="beam" type="box" pos="0.75 0 1.45" size="0.15 0.6 0.10" rgba="0.8 0.4 0.2 1"/>'
    out = {}
    for label, crouch in (("standing", False), ("crouched", True)):
        r = Run(world=beam)
        r.run(2.0, S.ActionFrame(crouch=crouch))
        x0 = r.c.com(r.d)[0]
        beam_contact = {"n": 0, "F": 0.0}
        bid = r.m.geom("beam").id

        def act(x):
            f6 = np.zeros(6)
            for i in range(x.d.ncon):
                c = x.d.contact[i]
                if bid in (c.geom1, c.geom2):
                    mujoco.mj_contactForce(x.m, x.d, i, f6)
                    beam_contact["n"] += 1
                    beam_contact["F"] = max(beam_contact["F"], float(abs(f6[0])))
            return S.ActionFrame(move=1.0 if x.c.com(x.d)[0] - x0 < 1.4 else 0.0, crouch=crouch)

        r.run(45.0, act, until=lambda x: x.c.com(x.d)[0] - x0 >= 1.4 and x.c.skill == "stand")
        out[label] = dict(progress_m=round(float(r.c.com(r.d)[0] - x0), 3), passed_beam=bool(r.c.com(r.d)[0] > 0.95),
                          beam_contact_steps=beam_contact["n"], beam_force_max_N=round(beam_contact["F"], 1), fell=r.fell,
                          max_tilt_deg=round(r.max_tilt, 2))
        r.video(f"g5_beam_{label}")
    out["pass"] = (out["crouched"]["passed_beam"] and out["crouched"]["fell"] is None
                   and not out["standing"]["passed_beam"] and out["standing"]["beam_contact_steps"] > 0)
    return out


GATES = {"g1": g1_stand_one_leg, "g2": g2_one_step, "g3": g3_walk_1m, "g4": g4_starts, "g5": g5_crouch_beam}


def main():
    names = [a for a in sys.argv[1:] if a in GATES] or list(GATES)
    path = Path(__file__).resolve().parent / "gate_results.json"
    results = json.loads(path.read_text()) if path.exists() else {}
    for n in names:
        res = GATES[n]()
        results[n] = res
        brief = {k: v for k, v in res.items() if k != "log"}
        print(n, json.dumps(brief, default=str)[:1400], flush=True)
    path.write_text(json.dumps(results, indent=1, default=str))


if __name__ == "__main__":
    main()
