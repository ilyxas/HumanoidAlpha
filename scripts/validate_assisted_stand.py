#!/usr/bin/env python3
"""Headless ASSISTED stand validation vs RAW zero fall contrast.

PASS: continuous upright (pelvis z >= 0.70 m, tilt <= 0.55 rad) for >= 5 s.
Reports best hold, time-to-fall, and whether ≥10 s target is met.
Writes reports/assisted_stand/{metrics.json,timeseries.csv,plot.png,stand_t0.png,stand_t5.png}.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "physics"))

import mujoco
from agent_api.env import HumanoidEnv
from assisted_stand import AssistedStandController

OUT = ROOT / "reports" / "assisted_stand"
UPRIGHT_Z = 0.70
UPRIGHT_TILT = 0.55
ASSIST_T = 15.0
RAW_T = 2.0
PASS_HOLD = 5.0


def pelvis_tilt(data, pelvis_id):
    w, x, y, z = data.xquat[pelvis_id]
    up_z = 1.0 - 2.0 * (x * x + y * y)
    return float(np.arccos(np.clip(up_z, -1.0, 1.0)))


def try_render(model, data, path: Path):
    try:
        renderer = mujoco.Renderer(model, 480, 640)
        renderer.update_scene(data, camera="track" if False else -1)
        # Use free camera looking at pelvis
        renderer.update_scene(data)
        path.write_bytes(b"")  # ensure parent exists
        import imageio  # optional
        imageio.imwrite(path, renderer.render())
        return True
    except Exception:
        try:
            renderer = mujoco.Renderer(model, 480, 640)
            renderer.update_scene(data)
            rgb = renderer.render()
            # Write PPM without extra deps
            h, w, _ = rgb.shape
            with open(path.with_suffix(".ppm"), "wb") as f:
                f.write(f"P6\n{w} {h}\n255\n".encode())
                f.write(rgb.tobytes())
            # Also try png via matplotlib
            try:
                import matplotlib
                matplotlib.use("Agg")
                import matplotlib.pyplot as plt
                plt.imsave(path, rgb)
                path.with_suffix(".ppm").unlink(missing_ok=True)
                return True
            except Exception:
                if path.with_suffix(".ppm").exists():
                    path.with_suffix(".ppm").replace(path)
                return path.exists()
        except Exception as e:
            print(f"render skip: {e}")
            return False


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    env = HumanoidEnv()
    m, d = env.model, env.data
    assist = AssistedStandController(m)
    pelvis = assist._pelvis

    # --- ASSISTED ---
    assist.reset_to_stand(m, d)
    assist.enabled = True
    u_cmd = np.zeros(m.nu)
    try_render(m, d, OUT / "stand_t0.png")

    rows = []
    best_hold = 0.0
    hold = 0.0
    time_to_fall = None  # first time upright criteria fail after t>0
    steps = int(ASSIST_T / m.opt.timestep)
    t5_captured = False
    t10_captured = False
    for step in range(steps):
        tau = assist.compute(m, d, u_cmd)
        d.ctrl[:] = tau
        mujoco.mj_step(m, d)
        mujoco.mj_forward(m, d)  # match control_console.tick
        z = float(d.xpos[pelvis, 2])
        tilt = pelvis_tilt(d, pelvis)
        ncon = assist._foot_contacts(d)
        diag = dict(assist.diagnostics)
        diag.update(pelvis_z=z, tilt=tilt, ncon_feet=ncon,
                    com_xy=d.subtree_com[pelvis, :2].copy(),
                    support_xy=assist._support_xy(d))
        upright = z >= UPRIGHT_Z and tilt <= UPRIGHT_TILT
        if upright:
            hold += m.opt.timestep
            best_hold = max(best_hold, hold)
        else:
            if time_to_fall is None and d.time > 1e-6:
                time_to_fall = float(d.time)
            hold = 0.0
        if step % 10 == 0:
            rows.append(
                dict(
                    t=float(d.time),
                    pelvis_z=z,
                    tilt=tilt,
                    ncon_feet=ncon,
                    com_x=float(diag["com_xy"][0]),
                    com_y=float(diag["com_xy"][1]),
                    support_x=float(diag["support_xy"][0]),
                    support_y=float(diag["support_xy"][1]),
                    tau_norm=diag["tau_norm"],
                    fade=diag["fade"],
                    upright=upright,
                )
            )
        if not t5_captured and d.time >= 5.0 - 1e-9:
            try_render(m, d, OUT / "stand_t5.png")
            t5_captured = True
        if not t10_captured and d.time >= 10.0 - 1e-9:
            try_render(m, d, OUT / "stand_t10.png")
            t10_captured = True

    assisted = dict(
        duration_s=ASSIST_T,
        best_continuous_upright_s=best_hold,
        time_to_fall_s=time_to_fall,
        final_pelvis_z=rows[-1]["pelvis_z"] if rows else None,
        final_tilt=rows[-1]["tilt"] if rows else None,
        final_fade=rows[-1]["fade"] if rows else None,
        pass_hold_s=PASS_HOLD,
        target_hold_s=10.0,
        meets_10s_target=bool(best_hold >= 10.0 - 1e-9),
        criteria=dict(pelvis_z_min=UPRIGHT_Z, tilt_max=UPRIGHT_TILT),
    )

    # --- RAW zero contrast ---
    mujoco.mj_resetData(m, d)
    assist.reset_to_stand(m, d)
    d.ctrl[:] = 0
    raw_rows = []
    for step in range(int(RAW_T / m.opt.timestep)):
        mujoco.mj_step(m, d)
        if step % 25 == 0:
            mujoco.mj_forward(m, d)
            raw_rows.append(
                dict(
                    t=float(d.time),
                    pelvis_z=float(d.xpos[pelvis, 2]),
                    tilt=pelvis_tilt(d, pelvis),
                    ncon_feet=assist._foot_contacts(d),
                )
            )
    mujoco.mj_forward(m, d)
    raw = dict(
        duration_s=RAW_T,
        final_pelvis_z=float(d.xpos[pelvis, 2]),
        final_tilt=pelvis_tilt(d, pelvis),
        samples=raw_rows,
    )

    passed = best_hold >= PASS_HOLD
    metrics = dict(
        result="PASS" if passed else "FAIL",
        assisted=assisted,
        raw_zero=raw,
        notes=(
            "ASSISTED uses posture PD + subtree-COM Jacobian balance; "
            "no MJCF changes; no qpos writes during sim."
            if passed
            else "Hold below 5 s — see ASSISTED_STAND_REPORT.md for limiting mechanism."
        ),
    )
    (OUT / "metrics.json").write_text(json.dumps(metrics, indent=2))

    # CSV
    import csv
    with (OUT / "timeseries.csv").open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()) if rows else ["t"])
        w.writeheader()
        w.writerows(rows)

    # Plot
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        t = [r["t"] for r in rows]
        fig, ax = plt.subplots(3, 1, figsize=(8, 7), sharex=True)
        ax[0].plot(t, [r["pelvis_z"] for r in rows], label="ASSISTED pelvis z")
        ax[0].axhline(UPRIGHT_Z, color="g", ls="--", label="upright z")
        ax[0].set_ylabel("pelvis z (m)")
        ax[0].legend(loc="best")
        ax[1].plot(t, [r["tilt"] for r in rows], label="tilt")
        ax[1].axhline(UPRIGHT_TILT, color="g", ls="--", label="upright tilt")
        ax[1].set_ylabel("tilt (rad)")
        ax[1].legend(loc="best")
        ax[2].plot(t, [r["ncon_feet"] for r in rows], label="foot contacts")
        if raw_rows:
            ax[0].plot(
                [r["t"] for r in raw_rows],
                [r["pelvis_z"] for r in raw_rows],
                "r:",
                label="RAW z (2 s)",
            )
            ax[0].legend(loc="best")
        ax[2].set_xlabel("time (s)")
        ax[2].set_ylabel("ncon feet")
        fig.suptitle(f"ASSISTED stand — {metrics['result']} (best hold {best_hold:.2f}s)")
        fig.tight_layout()
        fig.savefig(OUT / "plot.png", dpi=120)
        plt.close(fig)
    except Exception as e:
        print(f"plot skip: {e}")

    print(json.dumps(metrics, indent=2))
    print(f"Wrote {OUT}")
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
