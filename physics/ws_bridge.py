#!/usr/bin/env python3
"""MuJoCo → WebSocket pose bridge for v0.2 browser viewer.

Runs physics at dt=0.002; broadcasts mapped poses at ~60 Hz (decoupled).
Scenarios: rest | knee | shoulder | fall | hold
No Blender in loop.
"""
from __future__ import annotations

import argparse
import asyncio
import json
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))

from agent_api.env import HumanoidEnv
from diagnostic_api.diag import HumanoidDiag
from bridge_full.mapping_full import extract_full_pose, load_calibration, clear_calibration_cache

try:
    import websockets
    from websockets.simple_server import serve  # may not exist
except Exception:
    serve = None
import websockets


def make_pose(diag: HumanoidDiag) -> dict:
    obs = diag.env.observe()
    pose = extract_full_pose(obs, diag.body_xpos(), diag.body_xquat())
    return pose


def action_zeros(env: HumanoidEnv) -> np.ndarray:
    return np.zeros(env.nu, dtype=np.float64)


def set_motor(env: HumanoidEnv, name: str, value: float, action: np.ndarray) -> None:
    try:
        i = env.model.actuator(name).id
    except Exception:
        # fallback by name list
        names = [
            __import__("mujoco").mj_id2name(env.model, __import__("mujoco").mjtObj.mjOBJ_ACTUATOR, i)
            for i in range(env.nu)
        ]
        if name not in names:
            raise KeyError(name)
        i = names.index(name)
    action[i] = value


async def bridge(host: str, port: int, scenario: str, broadcast_hz: float):
    clear_calibration_cache()
    calib = load_calibration(ROOT / "bridge_full" / "calibration_rest.json")
    env = HumanoidEnv(model_path=ROOT / "model" / "humanoid_alpha.xml")
    diag = HumanoidDiag(env)
    env.reset()
    dt = env.timestep
    assert abs(dt - 0.002) < 1e-9, dt

    clients = set()
    scenario_t0 = time.perf_counter()
    phys_steps = 0
    last_hz_t = time.perf_counter()
    phys_hz = 0.0
    steps_window = 0

    async def handler(ws):
        clients.add(ws)
        try:
            await ws.send(json.dumps({
                "type": "hello",
                "scenario": scenario,
                "dt": dt,
                "nq": int(env.model.nq),
                "nv": int(env.model.nv),
                "nu": int(env.model.nu),
            }))
            async for _ in ws:
                pass
        finally:
            clients.discard(ws)

    async def physics_loop():
        nonlocal phys_steps, phys_hz, steps_window, last_hz_t, scenario_t0
        broadcast_period = 1.0 / broadcast_hz
        last_broadcast = time.perf_counter()
        ctrl = action_zeros(env)

        # Wait briefly for a client so early frames are capturable
        for _ in range(500):
            if clients:
                break
            await asyncio.sleep(0.02)

        if scenario == "fall":
            env.reset(initial_state={"root_pos": [0.0, 0.0, 1.25]})
            scenario_t0 = time.perf_counter()
            phys_steps = 0
        elif scenario == "posed":
            # knee flexed ~ -1.4 rad (FACT local), slight shoulder; then hold via forward only
            env.reset(initial_state={
                "hinge_qpos": {"knee_r": -1.4, "knee_l": -0.2},
            })
            import mujoco
            mujoco.mj_forward(env.model, env.data)

        while True:
            t_sc = time.perf_counter() - scenario_t0
            ctrl[:] = 0.0

            if scenario == "knee":
                # Controlled knee flexion torque pulses
                set_motor(env, "knee_r_motor", -40.0 if (int(t_sc * 2) % 2 == 0) else 10.0, ctrl)
            elif scenario == "shoulder":
                set_motor(env, "shoulder_r_x_motor", 30.0 * np.sin(t_sc * 2.0), ctrl)
                set_motor(env, "shoulder_r_z_motor", 15.0 * np.cos(t_sc * 1.5), ctrl)
            elif scenario == "fall":
                ctrl[:] = 0.0
            elif scenario == "posed":
                # static MuJoCo FK pose (no stepping) — still real mj transforms
                ctrl[:] = 0.0
            elif scenario == "hold":
                ctrl[:] = 0.0
            # rest: zeros after reset

            env.data.ctrl[:] = ctrl
            if scenario == "posed":
                import mujoco
                mujoco.mj_forward(env.model, env.data)
            else:
                env.step()
            phys_steps += 1
            steps_window += 1

            now = time.perf_counter()
            if now - last_hz_t >= 1.0:
                phys_hz = steps_window / (now - last_hz_t)
                steps_window = 0
                last_hz_t = now

            if now - last_broadcast >= broadcast_period:
                last_broadcast = now
                if clients:
                    pose = make_pose(diag)
                    obs = env.observe()
                    msg = {
                        "type": "pose",
                        "sim_time": float(env.data.time),
                        "phys_hz": phys_hz,
                        "ncon": int(env.data.ncon),
                        "floor_contact": bool(obs["contact"]["floor_contact"]),
                        "t_send_ms": int(time.time() * 1000),
                        "pose": {
                            "mode": pose["mode"],
                            "root_pos": pose["root_pos"],
                            "root_quat_wxyz": pose["root_quat_wxyz"],
                            "apply_root_translate": True,
                            "bone_world_quat": pose["bone_world_quat"],
                            "bone_world_pos": pose["bone_world_pos"],
                            "apply_order": pose["apply_order"],
                        },
                    }
                    raw = json.dumps(msg)
                    dead = []
                    for c in list(clients):
                        try:
                            await c.send(raw)
                        except Exception:
                            dead.append(c)
                    for c in dead:
                        clients.discard(c)

            # Pace realtime (best-effort); fall may run faster for demo
            await asyncio.sleep(dt * (0.05 if scenario == "fall" else 1.0))

    print(f"WS bridge on ws://{host}:{port} scenario={scenario} dt={dt}", flush=True)
    async with websockets.serve(handler, host, port):
        await physics_loop()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--scenario", default="knee",
                    choices=["rest", "knee", "shoulder", "fall", "hold", "posed"])
    ap.add_argument("--broadcast-hz", type=float, default=60.0)
    args = ap.parse_args()
    # Quick self-check actuators
    env = HumanoidEnv(model_path=ROOT / "model" / "humanoid_alpha.xml")
    import mujoco
    acts = [mujoco.mj_id2name(env.model, mujoco.mjtObj.mjOBJ_ACTUATOR, i) for i in range(env.nu)]
    print("actuators:", acts, flush=True)
    print("nq nv nu", env.model.nq, env.model.nv, env.model.nu, flush=True)
    asyncio.run(bridge(args.host, args.port, args.scenario, args.broadcast_hz))


if __name__ == "__main__":
    main()
