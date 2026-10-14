"""Interactive MuJoCo window for Xandra-Lite (primitives only). macOS needs mjpython:

  .venv/bin/mjpython experiments/lite/play.py [--variant qp_v0|qp_v1|baseline|servo_v1]

Keys (chosen to avoid the MuJoCo window's own shortcuts):
  Up arrow     walk forward (stays on)        Down arrow   stop (closing step, then stand)
  Page Down    crouch on/off                  Home         reset to the start stance (logged seed)
The body is driven only through ActionFrame, exactly as in the gate tests. Mouse: drag to orbit,
right-drag to pan, scroll to zoom; Space pauses (MuJoCo).
"""
from __future__ import annotations

import sys
import time
from pathlib import Path

import mujoco
import mujoco.viewer

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "physics"))
from lite import servo_control as S  # noqa: E402
from lite.model import build  # noqa: E402

VARIANTS = {"baseline": ("servo", "v0"), "servo_v1": ("servo", "v1"), "qp_v0": ("qp", "v0"), "qp_v1": ("qp", "v1")}
KEY_UP, KEY_DOWN, KEY_PAGE_DOWN, KEY_HOME = 265, 264, 267, 268


def main():
    variant = sys.argv[sys.argv.index("--variant") + 1] if "--variant" in sys.argv else "qp_v0"
    ctrl, geom = VARIANTS[variant]
    m = build(servo=(ctrl == "servo"), geometry=geom)
    d = mujoco.MjData(m)
    if ctrl == "qp":
        from lite.qp_control import QPController
        c = QPController(m)
    else:
        c = S.ServoController(m)
    c.reset(d)
    state = dict(move=0.0, crouch=False, reset=False)

    def on_key(key):
        if key == KEY_UP:
            state["move"] = 1.0
        elif key == KEY_DOWN:
            state["move"] = 0.0
        elif key == KEY_PAGE_DOWN:
            state["crouch"] = not state["crouch"]
        elif key == KEY_HOME:
            state["reset"] = True

    print(f"Xandra-Lite {variant} ({ctrl}, geometry {geom}). Up = walk, Down = stop, PageDown = crouch, Home = reset.")
    with mujoco.viewer.launch_passive(m, d, key_callback=on_key) as v:
        v.cam.distance, v.cam.azimuth, v.cam.elevation = 3.5, 60, -15
        wall0, sim0 = time.perf_counter(), d.time
        last_print = 0.0
        while v.is_running():
            if state["reset"]:
                state.update(reset=False, move=0.0, crouch=False)
                c.reset(d)
                wall0, sim0 = time.perf_counter(), d.time
            # advance simulated time to wall-clock time (never more than 50 ms per frame)
            target = sim0 + (time.perf_counter() - wall0)
            budget = time.perf_counter() + 0.05
            with v.lock():
                while d.time < target and time.perf_counter() < budget:
                    c.step(d, S.ActionFrame(move=state["move"], crouch=state["crouch"]))
                    mujoco.mj_step(m, d)
                v.cam.lookat[:] = [d.subtree_com[c.torso, 0], d.subtree_com[c.torso, 1], 0.8]
            if d.time < target - 0.2:  # running slower than real time: let the clock follow
                wall0, sim0 = time.perf_counter(), d.time
            if time.perf_counter() - last_print > 1.0:
                last_print = time.perf_counter()
                print(f"\rt={d.time:6.1f}s  skill={c.skill:5s} support={c.support:8s} move={state['move']:.0f} "
                      f"crouch={state['crouch']!s:5s} x={d.subtree_com[c.torso, 0]:+.2f} m", end="", flush=True)
            v.sync()
            time.sleep(0.005)


if __name__ == "__main__":
    main()
