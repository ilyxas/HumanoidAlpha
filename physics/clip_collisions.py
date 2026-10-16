"""Collision audit of animation clips on the physical Xandra body. No physics stepping.

Every frame of every clip in a GLB puts the MuJoCo collision shapes exactly where the clip's
bones are, then runs mj_collision only: no dynamics, no motors, no controller, no joints. Each
body takes its lead bone's world transform (the hulls were cut from the rest skin by bone
weights), so the shapes follow the animated skin even where the 33-motor skeleton cannot reach
the pose (1-DoF elbows/knees lose forearm twist: joint retargeting put the hands 10-15 cm off).
The result shows where the *animation itself* makes the body pass through itself or the floor.

Reported per contact (penetration only, dist < 0 - the 6 mm contact margin is not a hit):
  a, b   - the two body parts (MuJoCo body names), b may be "floor"
  pa, pb - the deepest point of each part's surface inside the other, world (x, y, z) in metres.
           World = MuJoCo = Blender frame: Z up, Xandra faces -Y, her left is +X.
           Clips are in place, so this is clip-local space (the Animation page adds root motion).
  depth  - penetration in mm (= |pa - pb|)
  kind   - "floor"; "joint" (neighbours across one joint: the child's distal guard hull against its
           parent, i.e. the joint bends the limb into its parent - hulls can overlap at the seam);
           "self" (any other two parts, e.g. hand-hand, thigh-thigh)
Consecutive frames of the same pair are merged into events (start..end, worst frame).
Joint limits: the clip is also retargeted to joint angles (clip_retarget) and every angle beyond
the model's joint range is reported - the physical body could not perform that pose.

Limits of the method: the collision shapes are convex hulls of the rest skin per segment
(hands are mittens, the hull bridges concave areas), so a hit means "skins meet or are very
close", not an exact skin-skin intersection; skin bending at joints and hair/clothes are not
modelled; parent-child neighbours (e.g. thigh-calf) are not checked.

CLI:
  .venv/bin/python physics/clip_collisions.py [GLB ...] [--fps 30] [--out DIR] [--min-depth-mm 1]
  default GLB: assets/Xandra_Animated.glb; writes <out>/<glb stem>.collisions.json + .md
  default out: reports/clip_collisions/ (the Animation page's COLLISIONS overlay reads it from there)
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from clip_retarget import GLB, Glb, Retargeter  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
MODEL = ROOT / "physics" / "model" / "humanoid_alpha.xml"
OUT = ROOT / "reports" / "clip_collisions"
FORMAT = "xandra-clip-collisions/1"


def load_model(path: Path = MODEL) -> mujoco.MjModel:
    return mujoco.MjModel.from_xml_path(str(path))


def contact_points(c) -> tuple[np.ndarray, np.ndarray]:
    """Points on geom1 / geom2 surfaces. pos is the midpoint, the normal points geom1 -> geom2."""
    n = c.frame[:3]
    return c.pos - n * c.dist / 2, c.pos + n * c.dist / 2


class ClipCollisions:
    def __init__(self, model: mujoco.MjModel | None = None, glb: Glb | None = None):
        self.m = model or load_model()
        self.d = mujoco.MjData(self.m)
        self.glb = glb or Glb()
        self.rt = Retargeter(self.m, self.glb)
        self.floor = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_GEOM, "floor")
        self.guards = {g for g in range(self.m.ngeom) if self.m.geom(g).name.endswith("_joint_guard")}
        self.bodies = [(self.m.body(b).id, b, bone) for b, bone in self.rt.body_to_bone.items()]
        self.bone_names = sorted({bone for _, _, bone in self.bodies})
        # Lead bone head in its body's frame, measured in the bind pose (bodies have identity rest
        # orientation there, so this offset is fixed for the whole clip).
        bq, pp = self.rt.body_quats(None, 0.0)
        self.d.qpos[:] = self.rt.qpos_from(bq, pp, clamp=False)
        mujoco.mj_kinematics(self.m, self.d)
        heads = self.bone_heads(None, 0.0)
        self.offset = {bid: self.d.xmat[bid].reshape(3, 3).T @ (heads[bone] - self.d.xpos[bid])
                       for bid, _, bone in self.bodies}

    def bone_heads(self, chans, t) -> dict:
        """Bone head positions, GLB Y-up -> world Z-up (same conversion as the pelvis root)."""
        return {n: np.array([p[0], -p[2], p[1]]) for n, (_q, p) in self.glb.world(self.bone_names, chans, t).items()}

    def part(self, geom: int) -> str:
        return "floor" if geom == self.floor else self.m.body(int(self.m.geom_bodyid[geom])).name

    def place(self, chans, t) -> None:
        """Body and geom world poses straight from the bones (joints are bypassed)."""
        m, d = self.m, self.d
        bq, _ = self.rt.body_quats(chans, t)
        heads = self.bone_heads(chans, t)
        for bid, body, bone in self.bodies:
            mat = np.zeros(9)
            mujoco.mju_quat2Mat(mat, bq[body])
            r = mat.reshape(3, 3)
            d.xquat[bid] = bq[body]
            d.xmat[bid] = mat
            d.xpos[bid] = heads[bone] - r @ self.offset[bid]
        for g in range(m.ngeom):
            b = int(m.geom_bodyid[g])
            if b == 0:
                continue
            gm = np.zeros(9)
            mujoco.mju_quat2Mat(gm, m.geom_quat[g])
            r = d.xmat[b].reshape(3, 3)
            d.geom_xpos[g] = d.xpos[b] + r @ m.geom_pos[g]
            d.geom_xmat[g] = (r @ gm.reshape(3, 3)).ravel()

    def hits(self) -> list[dict]:
        """Penetrating contacts of the placed pose, deepest per body pair."""
        mujoco.mj_collision(self.m, self.d)
        best: dict[tuple, dict] = {}
        for i in range(self.d.ncon):
            c = self.d.contact[i]
            if c.dist >= 0:
                continue
            pa, pb = contact_points(c)
            a, b = self.part(c.geom1), self.part(c.geom2)
            kind = "floor" if self.floor in (c.geom1, c.geom2) else "joint" if (c.geom1 in self.guards or c.geom2 in self.guards) else "self"
            if (a == "floor") or (b != "floor" and b < a):  # stable order, floor always second
                a, b, pa, pb = b, a, pb, pa
            key = (a, b)
            depth = -float(c.dist) * 1000
            if key not in best or depth > best[key]["depth_mm"]:
                best[key] = dict(a=a, b=b, kind=kind, pa=np.round(pa, 4).tolist(), pb=np.round(pb, 4).tolist(),
                                 depth_mm=round(depth, 1))
        return sorted(best.values(), key=lambda h: -h["depth_mm"])

    def at(self, chans, t) -> list[dict]:
        self.place(chans, t)
        return self.hits()

    def clip(self, name: str, fps: float = 30.0, min_depth_mm: float = 1.0) -> dict:
        chans = self.glb.channels(name)
        dur = max(float(tr[0][-1]) for ch in chans.values() for tr in ch.values())
        ts = np.arange(0.0, dur + 1e-9, 1.0 / fps)
        frames, limits = [], {}
        for t in ts:
            hits = [h for h in self.at(chans, t) if h["depth_mm"] >= min_depth_mm]
            frames.append(dict(t=round(float(t), 4), hits=hits))
            log: list = []
            self.rt.qpos_from(*self.rt.body_quats(chans, t), clamp_log=log)
            for joint, excess in log:
                c = limits.setdefault(joint, dict(joint=joint, frames=0, max_deg=0.0, t_max=0.0))
                c["frames"] += 1
                if excess > c["max_deg"]:
                    c["max_deg"], c["t_max"] = round(excess, 1), round(float(t), 4)
        return dict(name=name, duration=round(dur, 4), fps=fps, loop=bool(self.rt.loops.get(name, False)), frames=frames,
                    events=events(frames), limits=sorted(limits.values(), key=lambda c: -c["max_deg"]))

    def all(self, fps: float = 30.0, min_depth_mm: float = 1.0) -> dict:
        return {n: self.clip(n, fps, min_depth_mm) for n in self.glb.anims}


def events(frames: list[dict]) -> list[dict]:
    """Merge consecutive frames of the same pair; keep the worst frame's points."""
    open_: dict[tuple, dict] = {}
    out = []
    for i, f in enumerate(frames):
        seen = set()
        for h in f["hits"]:
            key = (h["a"], h["b"])
            seen.add(key)
            e = open_.get(key)
            if e is None:
                e = open_[key] = dict(a=h["a"], b=h["b"], kind=h["kind"], start=f["t"], end=f["t"], frames=0, worst=None, _last=i)
            e["end"], e["frames"], e["_last"] = f["t"], e["frames"] + 1, i
            if e["worst"] is None or h["depth_mm"] > e["worst"]["depth_mm"]:
                e["worst"] = dict(t=f["t"], depth_mm=h["depth_mm"], pa=h["pa"], pb=h["pb"])
        for key in [k for k in open_ if k not in seen]:
            out.append(open_.pop(key))
    out += open_.values()
    for e in out:
        e.pop("_last", None)
    return sorted(out, key=lambda e: (e["start"], -e["worst"]["depth_mm"]))


def fmt_p(p) -> str:
    return "(" + ", ".join(f"{v:+.3f}" for v in p) + ")"


def markdown(glb_name: str, clips: dict) -> str:
    lines = [f"# Collisions in {glb_name}", "",
             "Collision shapes of the MuJoCo Xandra body placed on the clip's bones (no physics step). Points are world metres,",
             "Z up (Blender frame), clip-local (clips are in place). `a`/`b` point = deepest point of that",
             "part inside the other. kind: self = two non-neighbouring parts, joint = limb bent into its parent",
             "(hulls may overlap at the seam), floor.", ""]
    for name, c in clips.items():
        lines.append(f"## {name} ({c['duration']:.2f} s, {len(c['frames'])} frames)")
        lines.append("")
        if not c["events"]:
            lines.append("No penetrations.")
        else:
            lines += ["| from–to, s | frames | a | b | kind | worst, mm | at, s | point on a (x,y,z) | point on b (x,y,z) |",
                      "|---|---|---|---|---|---|---|---|---|"]
            for e in c["events"]:
                w = e["worst"]
                lines.append(f"| {e['start']:.2f}–{e['end']:.2f} | {e['frames']} | {e['a']} | {e['b']} | {e['kind']} | {w['depth_mm']:.1f}"
                             f" | {w['t']:.2f} | {fmt_p(w['pa'])} | {fmt_p(w['pb'])} |")
        if c["limits"]:
            lines += ["", "Beyond the model's joint range (the physical body could not do this): " + ", ".join(
                f"{k['joint']} +{k['max_deg']:.0f}° ({k['frames']} fr, worst {k['t_max']:.2f} s)" for k in c["limits"])]
        lines.append("")
    return "\n".join(lines)


def run(glb_path: Path, out_dir: Path, fps: float, min_depth_mm: float, model=None) -> dict:
    cc = ClipCollisions(model, Glb(glb_path))
    clips = cc.all(fps, min_depth_mm)
    doc = dict(format=FORMAT, glb=glb_path.name, frame="world, metres, Z up (MuJoCo = Blender), clip-local",
               min_depth_mm=min_depth_mm, clips=clips)
    out_dir.mkdir(parents=True, exist_ok=True)
    stem = glb_path.name[:-4] if glb_path.name.lower().endswith(".glb") else glb_path.name
    (out_dir / f"{stem}.collisions.json").write_text(json.dumps(doc, separators=(",", ":")))
    (out_dir / f"{stem}.collisions.md").write_text(markdown(glb_path.name, clips))
    return doc


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("glb", nargs="*", type=Path, default=[GLB])
    ap.add_argument("--fps", type=float, default=30.0)
    ap.add_argument("--min-depth-mm", type=float, default=1.0)
    ap.add_argument("--out", type=Path, default=OUT, help="output dir (default: reports/clip_collisions, read by the Animation page)")
    a = ap.parse_args(argv)
    model = load_model()
    for g in a.glb:
        doc = run(g, a.out, a.fps, a.min_depth_mm, model)
        for name, c in doc["clips"].items():
            worst = max((e["worst"]["depth_mm"] for e in c["events"]), default=0.0)
            print(f"{g.name} {name:10s} events={len(c['events']):3d} worst={worst:6.1f} mm limits={len(c['limits'])}")


if __name__ == "__main__":
    main()
