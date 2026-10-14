"""Xandra-Lite runtime for ./START.command lite (EXPERIMENTAL, branch exp/assisted-stand-claude).

One process owns MuJoCo; the browser page (viewer/lite.html) renders the model's primitives and sends
commands over a local WebSocket. Same ownership/timing pattern as physics/control_console.py:

  inventory (on connect)  model geometry (bodies, geoms) + 11 actuators (Harness stick format)
  state     (60 Hz)       body poses, skill, support, assist, foot forces, ctrl
  commands  {"op":"human_cmd","u":[11],"t"}  joystick torques (like Harness), zeroed after 200 ms
            {"op":"action","move":0|1,"crouch":bool}  skill input (ActionFrame)
            {"op":"assist","value":bool}            balance controller on/off (off = raw torques)
            {"op":"reset"}                           re-seed the standing pose (logged direct write)

No external forces on the body. Controller: QP (needs osqp in .lite_deps, see qp_control.py).
"""
from __future__ import annotations

import argparse
import asyncio
import json
import math
import os
import sys
import time
from pathlib import Path

import mujoco
import numpy as np
import websockets

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lite.model import ACTUATORS, build  # noqa: E402
from lite.servo_control import ActionFrame  # noqa: E402

HUMAN_CMD_STALE_S = 0.200
STATE_HZ = 60.0
QP_EVERY = 2  # QP at 250 Hz, torque held in between (verified: stand, step, 1 m, long walk)
# Fastest quasi-static profile that walked 40 s without a fall (0.10 m/s; the gate default is 0.08 m/s).
# Longer steps (0.25-0.35 m) fell after 2-6 steps: each step waits for the COM to settle over the foot.
GAIT = dict(step=0.20, swing=0.5, shift=0.6)
GEOM_TYPES = {int(mujoco.mjtGeom.mjGEOM_PLANE): "plane", int(mujoco.mjtGeom.mjGEOM_SPHERE): "sphere",
              int(mujoco.mjtGeom.mjGEOM_CAPSULE): "capsule", int(mujoco.mjtGeom.mjGEOM_BOX): "box",
              int(mujoco.mjtGeom.mjGEOM_CYLINDER): "cylinder"}


class LiteRuntime:
    def __init__(self, geometry: str = "v0"):
        try:
            from lite.qp_control import QPController
        except ImportError as error:  # osqp lives in .lite_deps of the experiment branch
            raise SystemExit(f"Xandra-Lite needs osqp in .lite_deps ({error}). "
                             "Install it there (pip --target .lite_deps osqp scipy jinja2 markupsafe --no-deps).")
        self.geometry = geometry
        self.m = build(servo=False, geometry=geometry)
        self.d = mujoco.MjData(self.m)
        self.c = QPController(self.m, qp_every=QP_EVERY, gait=GAIT)
        self.c.reset(self.d)
        self.action = ActionFrame()
        self.human_t = None
        self.revision = 0

    # ---------------------------------------------------------------- protocol
    def inventory(self):
        m = self.m
        bodies = [dict(id=b, name=m.body(b).name) for b in range(m.nbody)]
        geoms = []
        for g in range(m.ngeom):
            t = GEOM_TYPES.get(int(m.geom_type[g]))
            if t is None:
                continue
            geoms.append(dict(name=m.geom(g).name, type=t, body=int(m.geom_bodyid[g]), size=m.geom_size[g].tolist(),
                              pos=m.geom_pos[g].tolist(), quat=m.geom_quat[g].tolist(), rgba=m.geom_rgba[g].tolist()))
        actuators = []
        for i, a in enumerate(ACTUATORS):
            aid = m.actuator(a).id
            actuators.append(dict(id=i, name=a, joint=a, ctrlrange=m.actuator_ctrlrange[aid].tolist()))
        return dict(model="xandra_lite", geometry=self.geometry, controller="qp", bodies=bodies, geoms=geoms,
                    actuators=actuators, timestep=float(m.opt.timestep))

    def state(self):
        d, c = self.d, self.c
        poses = [np.r_[d.xpos[b], d.xquat[b]].round(5).tolist() for b in range(self.m.nbody)]
        return dict(type="state", t=round(float(d.time), 3), poses=poses, skill=c.skill, support=c.support,
                    assist=bool(c.assist), move=self.action.move, crouch=self.action.crouch,
                    com=d.subtree_com[c.torso].round(4).tolist(),
                    feet_N=[round(c.foot_force(d, 0), 1), round(c.foot_force(d, 1), 1)],
                    ctrl=d.ctrl[c.act].round(2).tolist(), human=c.u_ext.round(2).tolist(), revision=self.revision)

    def handle(self, cmd):
        op = cmd.get("op")
        if op == "human_cmd":
            u = cmd.get("u")
            if not isinstance(u, list) or len(u) != len(ACTUATORS):
                raise ValueError(f"human_cmd needs {len(ACTUATORS)} numbers")
            v = np.array([float(x) for x in u])
            if not np.isfinite(v).all():
                raise ValueError("human_cmd values must be finite")
            self.c.u_ext[:] = np.clip(v, -self.c.lim, self.c.lim)
            self.human_t = time.perf_counter()
        elif op == "action":
            move = float(cmd.get("move", 0.0))
            if not math.isfinite(move):
                raise ValueError("move must be finite")
            self.action = ActionFrame(move=max(-1.0, min(1.0, move)), crouch=bool(cmd.get("crouch", False)))
        elif op == "assist":
            self.c.assist = bool(cmd.get("value"))
        elif op == "reset":
            self.c.reset(self.d)
            self.c.u_ext[:] = 0
            self.action = ActionFrame()
        else:
            raise ValueError("unknown op")
        self.revision += 1

    def poll_stale(self):
        if self.human_t is not None and time.perf_counter() - self.human_t > HUMAN_CMD_STALE_S:
            self.c.u_ext[:] = 0
            self.human_t = None

    def tick(self):
        self.c.step(self.d, self.action)
        mujoco.mj_step(self.m, self.d)
        if not np.isfinite(self.d.qpos).all():
            self.c.reset(self.d)


async def serve(port: int, geometry: str):
    rt = LiteRuntime(geometry)
    inventory = rt.inventory()
    clients = set()
    owner = None

    async def send(ws, msg):
        try:
            await asyncio.wait_for(ws.send(json.dumps(msg, allow_nan=False)), timeout=0.5)
        except (websockets.ConnectionClosed, TimeoutError):
            pass

    async def handler(ws):
        # Single-user local tool: the newest page takes control (a reload must not end up read-only);
        # the previous owner is told it is now read-only and its inputs are cleared.
        nonlocal owner
        clients.add(ws)
        previous, owner = owner, ws
        rt.c.u_ext[:] = 0
        rt.action = ActionFrame()
        if previous is not None and previous is not ws:
            await send(previous, dict(type="control", can_control=False))
        await send(ws, dict(type="inventory", inventory=inventory, can_control=True))
        try:
            async for raw in ws:
                try:
                    cmd = json.loads(raw)
                    if not isinstance(cmd, dict):
                        raise ValueError("expected object")
                    if ws is not owner:
                        if cmd.get("op") == "human_cmd":
                            continue  # the stick pump of a read-only page runs at 50 Hz: ignore quietly
                        raise ValueError("read-only client")
                    rt.handle(cmd)
                except (ValueError, TypeError) as error:
                    await send(ws, dict(type="error", message=str(error)))
        finally:
            clients.discard(ws)
            if owner is ws:
                owner = next(iter(clients), None)
                rt.c.u_ext[:] = 0
                rt.action = ActionFrame()
                if owner is not None:
                    await send(owner, dict(type="control", can_control=True))

    async with websockets.serve(handler, "127.0.0.1", port, max_size=16384,
                                origins=[None, "http://127.0.0.1:8788", "http://localhost:8788"],
                                ping_interval=10, ping_timeout=10):
        print(f"Xandra-Lite WebSocket: ws://127.0.0.1:{port} (geometry {geometry}, QP)", flush=True)
        last = time.perf_counter()
        broadcast = last
        acc = 0.0
        dt = rt.m.opt.timestep
        parent = os.getppid()
        while True:
            # The launcher may only kill a wrapper process (framework Python on macOS starts the real
            # interpreter as a child); if our parent disappears, stop instead of holding the ports.
            if os.getppid() != parent:
                print("Xandra-Lite: launcher gone, stopping", flush=True)
                return
            now = time.perf_counter()
            acc += min(now - last, 0.05)  # never integrate a huge backlog
            last = now
            rt.poll_stale()
            while acc >= dt:
                rt.tick()
                acc -= dt
            if now - broadcast >= 1.0 / STATE_HZ and clients:
                msg = rt.state()
                await asyncio.gather(*(send(ws, msg) for ws in tuple(clients)))
                broadcast = now
            await asyncio.sleep(0.002)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--port", type=int, default=8766)
    p.add_argument("--geometry", choices=["v0", "v1"], default="v0")
    a = p.parse_args()
    asyncio.run(serve(a.port, a.geometry))


if __name__ == "__main__":
    main()
