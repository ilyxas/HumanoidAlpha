#!/usr/bin/env python3
"""Modest CEM retune of ASSISTED stand gains around current defaults.

Objective: maximize continuous upright hold (pelvis_z>=0.70, tilt<=0.55)
over a 15 s horizon. Prefer modest scale factors (not saturating forever).
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

UPRIGHT_Z = 0.70
UPRIGHT_TILT = 0.55
HORIZON = 15.0

# Base = current CEM defaults (approx)
BASE = dict(
    ankle=140.11521022913348,
    knee=289.1074908649772,
    hip=126.39406153252183,
    lumbar=51.92425113916003,
    thoracic=35.795525778161306,
    kp_com=3742.9945680711958,
    kd_com=129.5173816768333,
    # relative kd ratios preserved from DEFAULT_GAINS
    kd_ankle=0.05,
    kd_knee=0.06,
    kd_hip=0.06,
    kd_lumbar=0.08,
    kd_thoracic=0.08,
)


def apply_gains(g: dict) -> None:
    thor = g["thoracic"]
    AssistedStandController.DEFAULT_GAINS = {
        "ankle": (g["ankle"], g["ankle"] * g["kd_ankle"]),
        "knee": (g["knee"], g["knee"] * g["kd_knee"]),
        "hip": (g["hip"], g["hip"] * g["kd_hip"]),
        "lumbar": (g["lumbar"], g["lumbar"] * g["kd_lumbar"]),
        "thoracic": (thor, thor * g["kd_thoracic"]),
        "neck": (thor * 0.4, thor * 0.04),
        "shoulder": (thor * 0.6, thor * 0.06),
        "elbow": (thor * 0.5, thor * 0.05),
        "wrist": (thor * 0.2, thor * 0.02),
    }
    AssistedStandController.KP_COM = float(g["kp_com"])
    AssistedStandController.KD_COM = float(g["kd_com"])


def evaluate(env: HumanoidEnv, g: dict, horizon: float = HORIZON) -> dict:
    apply_gains(g)
    m, d = env.model, env.data
    assist = AssistedStandController(m)
    assist.reset_to_stand(m, d)
    assist.enabled = True
    u_cmd = np.zeros(m.nu)
    pelvis = assist._pelvis
    best_hold = 0.0
    hold = 0.0
    fall_t = horizon
    steps = int(horizon / m.opt.timestep)
    sat_count = 0
    for step in range(steps):
        tau = assist.compute(m, d, u_cmd)
        # saturation fraction
        lo, hi = assist._ctrl_lo, assist._ctrl_hi
        sat_count += int(np.any((tau <= lo + 1e-9) | (tau >= hi - 1e-9)))
        d.ctrl[:] = tau
        mujoco.mj_step(m, d)
        mujoco.mj_forward(m, d)
        z = float(d.xpos[pelvis, 2])
        tilt = assist._pelvis_tilt(d)
        upright = z >= UPRIGHT_Z and tilt <= UPRIGHT_TILT
        if upright:
            hold += m.opt.timestep
            best_hold = max(best_hold, hold)
        else:
            if hold > 0 and fall_t == horizon:
                # first loss of upright after having held
                pass
            hold = 0.0
            if z < 0.35 or tilt > 1.0:
                fall_t = float(d.time)
                break
    return dict(
        best_hold=best_hold,
        fall_t=fall_t,
        final_z=float(d.xpos[pelvis, 2]),
        final_tilt=assist._pelvis_tilt(d),
        sat_frac=sat_count / max(1, step + 1),
        gains=g,
    )


def sample(mean: dict, std_scale: float, rng: np.random.Generator) -> dict:
    g = {}
    # multiplicative log-normal-ish scales around mean
    for k, v in mean.items():
        if k.startswith("kd_"):
            # keep kd ratios near base with small noise
            g[k] = float(np.clip(v * np.exp(rng.normal(0, 0.15 * std_scale)), 0.02, 0.20))
        else:
            g[k] = float(np.clip(v * np.exp(rng.normal(0, 0.35 * std_scale)), v * 0.4, v * 2.5))
    return g


def main():
    env = HumanoidEnv()
    rng = np.random.default_rng(42)
    out = ROOT / "reports" / "assisted_stand"
    out.mkdir(parents=True, exist_ok=True)

    # Baseline
    base_res = evaluate(env, dict(BASE))
    print(f"BASE best_hold={base_res['best_hold']:.3f}s fall_t={base_res['fall_t']:.3f} sat={base_res['sat_frac']:.3f}")

    mean = dict(BASE)
    best = base_res
    history = [base_res]

    # CEM: 4 iterations, population 24, elite 6 — modest search
    pop, elite_n, iters = 24, 6, 5
    for it in range(iters):
        std_scale = 1.0 - 0.15 * it
        results = []
        for i in range(pop):
            g = sample(mean, std_scale, rng)
            # Always include mean itself once
            if i == 0:
                g = dict(mean)
            r = evaluate(env, g)
            results.append(r)
            print(f"  it{it} [{i:02d}] hold={r['best_hold']:.3f} fall={r['fall_t']:.2f} sat={r['sat_frac']:.2f}")
        results.sort(key=lambda r: (r["best_hold"], -r["sat_frac"]), reverse=True)
        elites = results[:elite_n]
        if elites[0]["best_hold"] > best["best_hold"] + 1e-6 or (
            abs(elites[0]["best_hold"] - best["best_hold"]) < 1e-6
            and elites[0]["sat_frac"] < best["sat_frac"]
        ):
            best = elites[0]
        # update mean from elites
        keys = list(mean.keys())
        for k in keys:
            mean[k] = float(np.mean([e["gains"][k] for e in elites]))
        history.append(dict(iter=it, best_hold=elites[0]["best_hold"], mean=dict(mean)))
        print(f"IT{it} elite_best={elites[0]['best_hold']:.3f} overall_best={best['best_hold']:.3f}")

    # Prefer gains that beat baseline meaningfully; else keep exploring best
    chosen = best
    print("CHOSEN", json.dumps({k: (round(v, 6) if isinstance(v, float) else v) for k, v in chosen.items() if k != 'gains'}, indent=2))
    print("GAINS", json.dumps(chosen["gains"], indent=2))

    (out / "tune_search.json").write_text(
        json.dumps(
            dict(baseline=base_res, best=chosen, history=history, mean_final=mean),
            indent=2,
            default=float,
        )
    )
    print(f"Wrote {out / 'tune_search.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
