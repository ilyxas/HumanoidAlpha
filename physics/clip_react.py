"""REACT: the animation drives the body, MuJoCo answers only when body parts hit each other.

EXPERIMENTAL, declared help (not for Game): the pelvis is a kinematic anchor that follows the clip
(mocap body), and every other part follows the clip through strong computed-torque drives. Without
contacts the pose equals the clip; when a hand runs into a thigh the contact pushes it away and the
drive brings it back to the clip afterwards. The result is compared against the original animation.

Settings (Ilya, 2026-10-16):
  self-collision response  on   - skin hulls of non-neighbouring parts push each other
  joint-limit enforcement  off  - no joint ranges, no joint guards (child-into-parent seam pairs)
  floor collision response off  - no floor; gravity off too (the pelvis anchor holds the body,
                                  nothing needs to balance)
  kinematic connectivity   kept - parts hang on joints at their lead bone's head:
       limbs (upper arm -> forearm -> hand, thigh -> calf -> foot, pelvis -> thigh, pelvis -> lower
       torso): ball joint, the clip keeps these bone lengths exactly;
       upper torso, head, upper arms: ball joint + 3 slides, because the unmodelled bones in
       between (spine_02-04, neck, clavicle) change that offset by up to 14 cm in the clips.
       Slides and balls are unlimited and driven to the clip like every other joint.

Not modelled: anything a contact does to the skin itself (the hulls are rigid), hair, clothes.

Output (reports/clip_react/<glb stem>.react.json, + .md):
  same frame format as clip_collisions (body poses, hulls) for the reacted pose; "hits" are the
  contacts that push (both points, depth, force_n); plus per frame "dev" {body: [position error mm, rotation error deg]} against the
  clip and per clip a summary. The Animation page's REACT toggle plays it on the skin.

CLI:
  .venv/bin/python physics/clip_react.py [GLB ...] [--clip NAME] [--hz 15] [--out DIR]
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from clip_collisions import ROOT, ClipCollisions, contact_points, events  # noqa: E402
from clip_retarget import GLB, Glb, qconj, qmul, qnorm, qrotvec  # noqa: E402

OUT = ROOT / "reports" / "clip_react"
FORMAT = "xandra-clip-react/1"
SLIDING = ("torso_upper", "head", "upperarm_l", "upperarm_r")  # offset to parent changes in clips
DT = 0.002


def build_model(cc: ClipCollisions) -> tuple[mujoco.MjModel, dict]:
    """REACT body: original masses and skin hulls, joints at lead-bone heads, no floor/limits."""
    src = cc.m
    bq, _ = cc.rt.body_quats(None, 0.0)
    heads = cc.bone_heads(None, 0.0)
    b2b = cc.rt.body_to_bone
    bind_xpos = {body: heads[bone] for bone, body in ((b2b[b], b) for b in b2b)}
    hull = {h["body"]: h for h in cc.hull_shapes()}
    offset = {body: cc.offset[src.body(body).id] for body in b2b}  # head in the original body frame
    parent = {body: src.body(int(src.body_parentid[src.body(body).id])).name for body in b2b}
    children: dict[str, list[str]] = {}
    for b, p in parent.items():
        children.setdefault(p, []).append(b)

    assets, f = [], lambda v: " ".join(f"{x:.6g}" for x in np.ravel(v))
    for body, h in hull.items():
        verts = np.asarray(h["verts"]) - offset[body]
        assets.append(f'<mesh name="{body}_hull" vertex="{f(verts)}"/>')

    def body_xml(name: str, depth: int) -> str:
        bid = src.body(name).id
        pos = bind_xpos[name] - (bind_xpos[parent[name]] if parent[name] != "world" else 0)
        attrs = f'name="{name}" pos="{f(pos)}"' + (' mocap="true"' if parent[name] == "world" else "")
        ipos = src.body_ipos[bid] - offset[name]
        inner = [f'<inertial pos="{f(ipos)}" quat="{f(src.body_iquat[bid])}" mass="{src.body_mass[bid]:.6g}" '
                 f'diaginertia="{f(src.body_inertia[bid])}"/>']
        if parent[name] != "world":
            if name in SLIDING:  # slides first: MuJoCo composes joints in order, so these move in the parent frame
                inner += [f'<joint name="{name}_s{a}" type="slide" axis="{ax}" damping="0.5"/>'
                          for a, ax in (("x", "1 0 0"), ("y", "0 1 0"), ("z", "0 0 1"))]
            inner.append(f'<joint name="{name}_ball" type="ball" damping="0.05"/>')
        if name in hull:
            inner.append(f'<geom name="{name}_geom" type="mesh" mesh="{name}_hull" margin="0.006"/>')
        elif name == "head":
            g = src.geom("head_geom").id
            inner.append(f'<geom name="head_geom" type="sphere" size="{src.geom_size[g][0]:.6g}" '
                         f'pos="{f(src.geom_pos[g] - offset[name])}" margin="0.006"/>')
        inner += [body_xml(c, depth + 1) for c in children.get(name, [])]
        pad = "  " * depth
        return f"{pad}<body {attrs}>\n" + "\n".join(pad + "  " + s for s in inner) + f"\n{pad}</body>"

    excludes = []
    for i in range(src.nexclude):
        sig = int(src.exclude_signature[i])
        a, b = src.body(sig >> 16).name, src.body(sig & 0xFFFF).name
        excludes.append(f'<exclude body1="{a}" body2="{b}"/>')
    xml = f"""<mujoco model="xandra_react">
  <compiler angle="radian" coordinate="local"/>
  <option timestep="{DT}" gravity="0 0 0" integrator="implicitfast" cone="elliptic"/>
  <default><geom condim="3" friction="0.8 0.1 0.1" solref="0.004 1" solimp="0.95 0.99 0.001"/></default>
  <asset>{''.join(assets)}</asset>
  <worldbody>
{body_xml("pelvis", 2)}
  </worldbody>
  <contact>{''.join(excludes)}</contact>
</mujoco>"""
    model = mujoco.MjModel.from_xml_string(xml)
    meta = dict(parent=parent, bind=bind_xpos, bind_rel={b: bind_xpos[b] - bind_xpos[parent[b]]
                                                         for b in b2b if parent[b] != "world"})
    return model, meta


class Reactor:
    """Kinematic pelvis + computed-torque tracking of the clip; contacts are the only disturbance."""

    def __init__(self, cc: ClipCollisions, hz: float = 15.0, zeta: float = 1.0):
        self.cc = cc
        self.m, self.meta = build_model(cc)
        self.d = mujoco.MjData(self.m)
        self.kp, self.kd = (2 * np.pi * hz) ** 2, 2 * zeta * 2 * np.pi * hz
        m = self.m
        self.bodies = [m.body(i).name for i in range(1, m.nbody)]
        self.ball = {b: m.joint(f"{b}_ball") for b in self.bodies if self.meta["parent"][b] != "world"}
        self.slides = {b: [m.joint(f"{b}_s{a}") for a in "xyz"] for b in SLIDING}
        self.mocap = m.body("pelvis").mocapid[0]

    def targets(self, chans, t):
        """Clip pose in this model's coordinates: pelvis pose, qpos, body world quats, bone heads."""
        bq, _ = self.cc.rt.body_quats(chans, t)
        heads = self.cc.bone_heads(chans, t)
        b2b, par = self.cc.rt.body_to_bone, self.meta["parent"]
        q = np.zeros(self.m.nq)
        for b, j in self.ball.items():
            a = j.qposadr[0]
            q[a:a + 4] = qnorm(qmul(qconj(bq[par[b]]), bq[b]))
        for b, js in self.slides.items():
            mat = np.zeros(9)
            mujoco.mju_quat2Mat(mat, bq[par[b]])
            rel = mat.reshape(3, 3).T @ (heads[b2b[b]] - heads[b2b[par[b]]])
            for k, j in enumerate(js):
                q[j.qposadr[0]] = rel[k] - self.meta["bind_rel"][b][k]
        return heads[b2b["pelvis"]], bq["pelvis"], q, bq, heads

    def reset(self, chans, t=0.0):
        """Logged reset seed: the clip's first pose, at rest."""
        p, r, q, _, _ = self.targets(chans, t)
        mujoco.mj_resetData(self.m, self.d)
        self.d.mocap_pos[self.mocap], self.d.mocap_quat[self.mocap] = p, r
        self.d.qpos[:] = q
        mujoco.mj_forward(self.m, self.d)

    def step(self, q_ref, qd_ref, pelvis):
        m, d = self.m, self.d
        d.mocap_pos[self.mocap], d.mocap_quat[self.mocap] = pelvis
        err = np.zeros(m.nv)
        for j in self.ball.values():
            a, v = j.qposadr[0], j.dofadr[0]
            err[v:v + 3] = qrotvec(qmul(qconj(d.qpos[a:a + 4]), q_ref[a:a + 4]))  # child-local, like qvel
        for js in self.slides.values():
            for j in js:
                err[j.dofadr[0]] = q_ref[j.qposadr[0]] - d.qpos[j.qposadr[0]]
        acc = self.kp * err + self.kd * (qd_ref - d.qvel)
        mujoco.mj_forward(m, d)  # contact-free M and bias at the current state
        tau = np.zeros(m.nv)
        mujoco.mj_mulM(m, d, tau, acc)
        d.qfrc_applied[:] = tau + d.qfrc_bias
        mujoco.mj_step(m, d)

    def hits(self, min_force_n=0.5):
        """Contacts that push (force >= min_force_n), deepest per body pair. depth_mm < 0 means the
        hulls are within the 6 mm contact margin but not yet overlapping: the reaction is working."""
        m, d, out = self.m, self.d, {}
        f6 = np.zeros(6)
        for i in range(d.ncon):
            c = d.contact[i]
            mujoco.mj_contactForce(m, d, i, f6)
            force = float(abs(f6[0]))
            if force < min_force_n:
                continue
            pa, pb = contact_points(c)
            a, b = m.body(int(m.geom_bodyid[c.geom1])).name, m.body(int(m.geom_bodyid[c.geom2])).name
            if b < a:
                a, b, pa, pb = b, a, pb, pa
            depth = -float(c.dist) * 1000
            key = (a, b)
            if key not in out:
                out[key] = dict(a=a, b=b, kind="self", pa=np.round(pa, 4).tolist(), pb=np.round(pb, 4).tolist(),
                                depth_mm=round(depth, 1), force_n=0.0)
            elif depth > out[key]["depth_mm"]:
                out[key].update(pa=np.round(pa, 4).tolist(), pb=np.round(pb, 4).tolist(), depth_mm=round(depth, 1))
            out[key]["force_n"] = round(out[key]["force_n"] + force, 1)
        return sorted(out.values(), key=lambda h: -h["force_n"])

    def clip(self, name, fps=30.0, min_force_n=0.5, baseline=True):
        """Reacted clip. baseline: also run it with contacts off, so the report separates the drives'
        own tracking lag from what the contacts did."""
        base = None
        if baseline:
            self.m.opt.disableflags |= int(mujoco.mjtDisableBit.mjDSBL_CONTACT)
            try:
                base = self.clip(name, fps, min_force_n, baseline=False)
            finally:
                self.m.opt.disableflags &= ~int(mujoco.mjtDisableBit.mjDSBL_CONTACT)
        cc, m, d = self.cc, self.m, self.d
        chans = cc.glb.channels(name)
        dur = max(float(tr[0][-1]) for ch in chans.values() for tr in ch.values())
        self.reset(chans)
        b2b = cc.rt.body_to_bone
        frames, n_out, t = [], int(np.floor(dur * fps + 1e-6)) + 1, 0.0
        prev = None
        for k in range(n_out):
            t_out = k / fps
            while True:
                pelvis_p, pelvis_r, q_ref, bq, heads = self.targets(chans, t)
                if prev is None:
                    prev = q_ref
                qd_ref = np.zeros(m.nv)
                mujoco.mj_differentiatePos(m, qd_ref, DT, prev, q_ref)
                prev = q_ref
                if t >= t_out - 1e-9:
                    break
                self.step(q_ref, qd_ref, (pelvis_p, pelvis_r))
                t += DT
            mujoco.mj_forward(m, d)
            dev, poses = {}, {}
            for b in self.bodies:
                bid = m.body(b).id
                pe = np.linalg.norm(d.xpos[bid] - heads[b2b[b]]) * 1000
                re = np.degrees(np.linalg.norm(qrotvec(qmul(qconj(bq[b]), d.xquat[bid]))))
                dev[b] = [round(float(pe), 1), round(float(re), 1)]
                # pose of the *original* body frame (same convention as clip_collisions), for the hulls
                mat = d.xmat[bid].reshape(3, 3)
                poses[b] = np.round(np.r_[d.xpos[bid] - mat @ cc.offset[cc.m.body(b).id], d.xquat[bid]], 4).tolist()
            frames.append(dict(t=round(t_out, 4), hits=self.hits(min_force_n), bodies=poses, dev=dev))
        worst = {b: [max(f["dev"][b][0] for f in frames), max(f["dev"][b][1] for f in frames)] for b in self.bodies}
        out = dict(name=name, duration=round(dur, 4), fps=fps, loop=bool(cc.rt.loops.get(name, False)),
                   frames=frames, events=events(frames), worst_dev=worst,
                   max_dev_mm=max(v[0] for v in worst.values()), max_dev_deg=max(v[1] for v in worst.values()))
        if base is not None:
            out["no_contact"] = dict(max_dev_mm=base["max_dev_mm"], max_dev_deg=base["max_dev_deg"], worst_dev=base["worst_dev"])
        return out


def markdown(glb_name, doc) -> str:
    lines = [f"# REACT: {glb_name}", "",
             f"Self-collision response on, joint limits off, floor off, gravity off; pelvis anchored to the clip;",
             f"drives {doc['settings']['hz']} Hz critically damped. Deviation = reacted pose vs the original clip",
             "(bone-head position mm, rotation deg).", ""]
    for name, c in doc["clips"].items():
        lines += [f"## {name} ({c['duration']:.2f} s)", "",
                  f"Max deviation: {c['max_dev_mm']:.0f} mm / {c['max_dev_deg']:.0f}° "
                  f"(drives alone, contacts off: {c['no_contact']['max_dev_mm']:.0f} mm / {c['no_contact']['max_dev_deg']:.0f}°).", "",
                  "| body | max mm | max deg | contacts off: mm | deg |", "|---|---|---|---|---|"]
        for b, (mm, deg) in sorted(c["worst_dev"].items(), key=lambda kv: -kv[1][0]):
            if mm >= 1 or deg >= 1:
                bm, bd = c["no_contact"]["worst_dev"][b]
                lines.append(f"| {b} | {mm:.0f} | {deg:.0f} | {bm:.0f} | {bd:.0f} |")
        if c["events"]:
            lines += ["", "Pushing contacts (force ≥ 0.5 N; depth < 0 = held apart inside the 6 mm margin):", "",
                      "| from–to, s | a | b | deepest, mm | at, s |", "|---|---|---|---|---|"]
            lines += [f"| {e['start']:.2f}–{e['end']:.2f} | {e['a']} | {e['b']} | {e['worst']['depth_mm']:.1f} | {e['worst']['t']:.2f} |"
                      for e in c["events"]]
        lines.append("")
    return "\n".join(lines)


def run(glb_path: Path, out_dir: Path, hz=15.0, only=None, fps=30.0):
    cc = ClipCollisions(glb=Glb(glb_path))
    r = Reactor(cc, hz)
    names = [only] if only else list(cc.glb.anims)
    clips = {n: r.clip(n, fps) for n in names}
    rest = {b: dict(bone=bone, quat_rest_wxyz=cc.rt.rest[bone].tolist()) for b, bone in cc.rt.body_to_bone.items()}
    doc = dict(format=FORMAT, glb=glb_path.name, frame="world, metres, Z up (MuJoCo = Blender), clip-local",
               settings=dict(self_collision=True, joint_limits=False, floor=False, gravity=False, hz=hz,
                             sliding=list(SLIDING)),
               rest=rest, hulls=cc.hull_shapes(), clips=clips)
    out_dir.mkdir(parents=True, exist_ok=True)
    stem = glb_path.name[:-4] if glb_path.name.lower().endswith(".glb") else glb_path.name
    (out_dir / f"{stem}.react.json").write_text(json.dumps(doc, separators=(",", ":")))
    (out_dir / f"{stem}.react.md").write_text(markdown(glb_path.name, doc))
    return doc


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("glb", nargs="*", type=Path, default=[GLB])
    ap.add_argument("--clip", default=None)
    ap.add_argument("--hz", type=float, default=15.0, help="drive bandwidth: higher = stiffer, less reaction")
    ap.add_argument("--out", type=Path, default=OUT)
    a = ap.parse_args(argv)
    for g in a.glb:
        doc = run(g, a.out, a.hz, a.clip)
        for name, c in doc["clips"].items():
            print(f"{g.name} {name:10s} max dev {c['max_dev_mm']:6.1f} mm {c['max_dev_deg']:5.1f} deg  events={len(c['events'])}")


if __name__ == "__main__":
    main()
