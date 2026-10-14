"""Xandra-Lite headless acceptance tests (MuJoCo primitives only; no clips, no skin, no browser).

  .venv/bin/python experiments/lite/run_tests.py [test ...] [--video DIR]

Every test drives the body ONLY through ActionFrame (the same input a human or an agent gives).
The only external forces are the declared test pushes (logged); the only direct state writes are
the seeds at reset (logged). Results -> experiments/lite/results.json.
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
from lite.control import ActionFrame, LiteController  # noqa: E402
from lite.model import build  # noqa: E402

VIDEO = Path(sys.argv[sys.argv.index("--video") + 1]) if "--video" in sys.argv else None


def tilt(d, body):
    return float(np.degrees(np.arccos(np.clip(d.xmat[body].reshape(3, 3)[2, 2], -1, 1))))


class Run:
    """One simulation: model + controller + optional video + fall detection."""

    def __init__(self, world="", start=(0.0, 0.0), grabbable=(), extra_actuators="", noise=0.0, seed=0, name="run"):
        self.m = build(world, extra_actuators, start, servo=False)  # path 1 drives torque motors
        self.d = mujoco.MjData(self.m)
        self.c = LiteController(self.m, grabbable)
        self.c.reset(self.d, joint_noise=noise, seed=seed)
        self.name = name
        self.frames = []
        self.max_tilt = 0.0
        self.fell = None
        if VIDEO:
            self.ren = mujoco.Renderer(self.m, 360, 640)
            self.cam = mujoco.MjvCamera()
            self.cam.distance, self.cam.azimuth, self.cam.elevation = 4.0, 90, -12

    def step(self, a: ActionFrame, push=None, extra=None):
        d = self.d
        self.c.step(d, a)
        if extra:
            extra(self)
        d.xfrc_applied[:] = 0
        if push is not None:
            d.xfrc_applied[self.c.torso, :3] = push
        mujoco.mj_step(self.m, d)
        t = tilt(d, self.c.torso)
        self.max_tilt = max(self.max_tilt, t)
        if self.fell is None and (t > 45 or d.xpos[self.c.torso, 2] < 0.35 + self.c.ground_z):
            self.fell = round(float(d.time), 2)
        if VIDEO and int(round(d.time / self.m.opt.timestep)) % 17 == 0:
            self.cam.lookat[:] = [d.subtree_com[self.c.torso, 0], 0, 0.9]
            self.ren.update_scene(d, self.cam)
            self.frames.append(self.ren.render().copy())

    def run(self, seconds, action, push_at=None, push=None, extra=None):
        dt = self.m.opt.timestep
        for _ in range(int(round(seconds / dt))):
            p = None
            if push_at is not None and push_at <= self.d.time < push_at + 0.1:
                p = push
            self.step(action(self) if callable(action) else action, p, extra)
        if push_at is not None:
            self.c.log.xfrc_events.append((push_at, push_at + 0.1, "torso", [float(v) for v in push]))

    def save_video(self, label):
        if not VIDEO or not self.frames:
            return
        VIDEO.mkdir(parents=True, exist_ok=True)
        h, w = self.frames[0].shape[:2]
        ff = subprocess.Popen(["ffmpeg", "-loglevel", "error", "-y", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{w}x{h}",
                               "-r", "29.4", "-i", "-", "-pix_fmt", "yuv420p", "-vcodec", "libx264", str(VIDEO / f"{label}.mp4")],
                              stdin=subprocess.PIPE)
        for f in self.frames:
            ff.stdin.write(f.tobytes())
        ff.stdin.close()
        ff.wait()

    def com(self):
        return self.d.subtree_com[self.c.torso].copy()


# ---------------------------------------------------------------- tests
def t1_stand_push():
    out = {}
    r = Run(name="stand")
    r.run(120.0, ActionFrame())
    out["hold_120s"] = dict(fell=r.fell, max_tilt_deg=round(r.max_tilt, 2), log=r.c.log.summary())
    for label, v in (("fwd", (1, 0, 0)), ("back", (-1, 0, 0)), ("side", (0, 1, 0))):
        r = Run(name=f"push_{label}")
        r.run(1.0, ActionFrame())
        r.run(5.0, ActionFrame(), push_at=1.0, push=np.array(v, float) * 200.0)
        out[f"push_20Ns_{label}"] = dict(fell=r.fell, max_tilt_deg=round(r.max_tilt, 2),
                                         com_moved_cm=round(float(np.linalg.norm(r.com()[:2])) * 100, 1),
                                         xfrc=r.c.log.xfrc_events)
        if label == "back":
            r.save_video("t1_push_back")
    out["pass"] = all(v.get("fell") is None for v in out.values() if isinstance(v, dict))
    return out


def t2_start_poses():
    res = []
    for s in range(5):
        r = Run(noise=0.06, seed=s, name=f"start{s}")
        r.run(10.0, ActionFrame())
        res.append(dict(seed=s, fell=r.fell, max_tilt_deg=round(r.max_tilt, 2), final_tilt_deg=round(tilt(r.d, r.c.torso), 2)))
    return dict(starts=res, pass_=all(x["fell"] is None for x in res))


def t3_walk_1m():
    r = Run(name="walk")
    x0 = r.com()[0]
    r.run(1.0, ActionFrame())

    def act(run):
        return ActionFrame(move=1.0 if run.com()[0] - x0 < 0.80 else 0.0)

    t_start = r.d.time
    r.run(20.0, act)
    x_end = r.com()[0] - x0
    hold_from = r.d.time
    r.run(5.0, ActionFrame())
    out = dict(distance_m=round(float(r.com()[0] - x0), 3), stopped_state=r.c.state, fell=r.fell,
               max_tilt_deg=round(r.max_tilt, 2), drift_while_holding_cm=round(float(abs(r.com()[0] - x0 - x_end)) * 100, 1),
               steps=sum(1 for e in r.c.log.skill_events if e[2] == "walk"), log=r.c.log.summary())
    out["pass"] = r.fell is None and 0.9 <= out["distance_m"] <= 1.2 and r.c.state == "stand"
    r.save_video("t3_walk_1m")
    return out


TESTS = {"t1": t1_stand_push, "t2": t2_start_poses, "t3": t3_walk_1m}


def main():
    names = [a for a in sys.argv[1:] if a in TESTS] or list(TESTS)
    path = Path(__file__).resolve().parent / "results.json"
    results = json.loads(path.read_text()) if path.exists() else {}
    for n in names:
        res = TESTS[n]()
        results[n] = res
        brief = {k: (v if not isinstance(v, dict) else {kk: vv for kk, vv in v.items() if kk != "log"}) for k, v in res.items() if k != "log"}
        print(n, json.dumps(brief, default=str)[:1500], flush=True)
    path.write_text(json.dumps(results, indent=1, default=str))


if __name__ == "__main__":
    main()
