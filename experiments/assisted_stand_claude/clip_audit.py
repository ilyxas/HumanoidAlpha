"""Physical playback audit of the 8 Xandra_Animated.glb clips on the stand-v2 model.

  .venv/bin/python experiments/assisted_stand_claude/clip_audit.py [--video DIR]

Each clip is played from the stand seed through StandV2Assist (clip -> targets, MuJoCo -> result),
loops for 2 cycles (at least 3 s), one-shots once plus 2 s. Reports: fell?, joint tracking error,
pelvis height error, lowest torque-feasibility scale, saturated actuators. Optional side-by-side
video per clip: retargeted clip pose (kinematic, left) vs physics (right).
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
from clip_retarget import Retargeter  # noqa: E402
from stand_v2 import StandV2Assist, load_flat_sole_model, tilt_deg  # noqa: E402


def joint_err_deg(m, q, qref, joints):
    e, out = np.zeros(3), []
    for j in joints:
        a = m.jnt_qposadr[j]
        if m.jnt_type[j] == mujoco.mjtJoint.mjJNT_BALL:
            mujoco.mju_subQuat(e, qref[a:a + 4], q[a:a + 4])
            out.append(np.linalg.norm(e))
        else:
            out.append(abs(qref[a] - q[a]))
    return np.degrees(np.sqrt(np.mean(np.square(out))))


def main():
    video_dir = Path(sys.argv[sys.argv.index("--video") + 1]) if "--video" in sys.argv else None
    m = load_flat_sole_model()
    clips = Retargeter(m).all_clips()
    joints = [j for j in range(m.njnt) if m.jnt_type[j] != mujoco.mjtJoint.mjJNT_FREE]
    results = {}
    for name, c in clips.items():
        d = mujoco.MjData(m)
        a = StandV2Assist(m)
        a.set_clips(clips)
        a.reset_to_stand(m, d)
        a.enabled = True
        settle = 0.5
        T = settle + (max(2 * c["duration"], 3.0) if c["loop"] else c["duration"] + 2.0)
        errs, zerr, alpha_min, sat, fell = [], [], 1.0, 0, None
        frames, ref = [], mujoco.MjData(m)
        if video_dir:
            m.vis.global_.offwidth, m.vis.global_.offheight = 640, 480
            ren = mujoco.Renderer(m, 480, 360)
            cam = mujoco.MjvCamera()
            cam.distance, cam.azimuth, cam.elevation = 3.4, 0, -12
        for k in range(int(T / m.opt.timestep)):
            if k == int(settle / m.opt.timestep):
                a.play(name)
            d.ctrl[:] = a.compute(m, d, np.zeros(m.nu))
            mujoco.mj_step(m, d)
            diag = a.diagnostics
            if a._clip is not None and d.time > settle + 0.4:
                errs.append(joint_err_deg(m, d.qpos, a.stance.q_des, joints))
                zerr.append(abs(d.qpos[2] - a.stance.z_des))
                alpha_min = min(alpha_min, a.stance.diag["alpha"])
                sat = max(sat, a.stance.diag["saturated"])
            if fell is None and diag.get("fallen"):
                fell = round(d.time - settle, 2)
            if video_dir and k % int(1 / (30 * m.opt.timestep)) == 0:
                ref.qpos[:] = a.stance.q_des
                ref.qpos[0:2] = d.qpos[0:2]
                mujoco.mj_forward(m, ref)
                img = []
                for dd in (ref, d):
                    cam.lookat[:] = [d.qpos[0], d.qpos[1], 0.75]
                    ren.update_scene(dd, cam)
                    img.append(ren.render().copy())
                frames.append(np.hstack(img))
        results[name] = dict(loop=c["loop"], sim_s=round(T, 2), fell_after_start_s=fell,
                             joint_rms_err_deg=round(float(np.mean(errs)), 1) if errs else None,
                             pelvis_z_err_cm=round(float(np.mean(zerr)) * 100, 1) if zerr else None,
                             min_torque_scale=round(alpha_min, 3), max_saturated=sat,
                             clip_flight_fraction=round(float((~c["stance"].any(1)).mean()), 2),
                             final_pelvis_z=round(float(d.qpos[2]), 3), final_xy=[round(float(v), 2) for v in d.qpos[:2]])
        print(f"{name:10s}", results[name], flush=True)
        if video_dir and frames:
            video_dir.mkdir(parents=True, exist_ok=True)
            out = video_dir / f"{name}.mp4"
            h, w = frames[0].shape[:2]
            ff = subprocess.Popen(["ffmpeg", "-loglevel", "error", "-y", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{w}x{h}",
                                   "-r", "30", "-i", "-", "-pix_fmt", "yuv420p", "-vcodec", "libx264", str(out)], stdin=subprocess.PIPE)
            for f in frames:
                ff.stdin.write(f.tobytes())
            ff.stdin.close()
            ff.wait()
    out = Path(__file__).resolve().parent / "clip_audit.json"
    out.write_text(json.dumps(results, indent=1))
    print("wrote", out.relative_to(ROOT))


if __name__ == "__main__":
    main()
