"""Side-by-side headless video: existing Assisted (repository model) vs StanceID (flat-sole variant).

  .venv/bin/python experiments/assisted_stand_claude/render_compare.py OUT.mp4 [seconds] [azimuth]

MuJoCo offscreen renderer (collision bodies, not the Xandra skin); frames piped to ffmpeg.
"""
from __future__ import annotations

import subprocess
import sys
from pathlib import Path

import mujoco
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from stand_lib import ROOT, StanceID, equilibrium_seed, load_model  # noqa: E402

sys.path.insert(0, str(ROOT / "physics"))
from assisted_stand import AssistedStandController  # noqa: E402

W, H, FPS = 480, 640, 30


class Old:
    def __init__(self):
        self.m = load_model("mesh")
        self.d = mujoco.MjData(self.m)
        self.c = AssistedStandController(self.m)
        self.c.reset_to_stand(self.m, self.d)
        self.c.enabled = True

    def step(self):
        self.d.ctrl[:] = self.c.compute(self.m, self.d, np.zeros(self.m.nu))
        mujoco.mj_step(self.m, self.d)


class New:
    def __init__(self):
        self.m = load_model("flat_sole")
        self.d = mujoco.MjData(self.m)
        equilibrium_seed(self.m, self.d)
        self.c = StanceID(self.m, cop="corners")
        self.c.reset(self.d)

    def step(self):
        self.d.ctrl[:] = self.c.compute(self.m, self.d)
        mujoco.mj_step(self.m, self.d)


def main():
    out = sys.argv[1]
    T = float(sys.argv[2]) if len(sys.argv) > 2 else 8.0
    az = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
    sims = [Old(), New()]
    for s in sims:
        s.m.vis.global_.offwidth, s.m.vis.global_.offheight = max(W, 640), max(H, 480)
    rens = [mujoco.Renderer(s.m, H, W) for s in sims]
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.lookat[:] = [0, 0, 0.8]
    cam.distance, cam.azimuth, cam.elevation = 3.2, az, -10
    ff = subprocess.Popen(["ffmpeg", "-loglevel", "error", "-y", "-f", "rawvideo", "-pix_fmt", "rgb24",
                           "-s", f"{2 * W}x{H}", "-r", str(FPS), "-i", "-", "-pix_fmt", "yuv420p",
                           "-vcodec", "libx264", out], stdin=subprocess.PIPE)
    dt = sims[0].m.opt.timestep
    per_frame = int(round(1 / (FPS * dt)))
    for _ in range(int(T * FPS)):
        frame = []
        for s, r in zip(sims, rens):
            for _ in range(per_frame):
                s.step()
            r.update_scene(s.d, cam)
            img = r.render().copy()
            img[:6, :, :] = (200, 60, 60) if s is sims[0] else (60, 170, 80)  # red = old, green = new
            frame.append(img)
        ff.stdin.write(np.hstack(frame).tobytes())
    ff.stdin.close()
    ff.wait()
    print("wrote", out)


if __name__ == "__main__":
    main()
