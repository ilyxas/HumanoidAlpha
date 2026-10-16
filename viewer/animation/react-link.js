// REACT toggle for the Animation mode (EXPERIMENTAL, declared help - not for Game).
// Plays the precomputed result of physics/clip_react.py on the skin: the clip drives the body,
// MuJoCo answers only to self-collisions (joint ranges, floor and gravity off; pelvis anchored to the
// clip). The 16 lead bones take the reacted body orientations; every other bone (spine, clavicles,
// neck, fingers) keeps the clip, so where nothing touches the skin equals the animation.
// Shows, per frame, how far the reacted pose is from the clip.
import * as THREE from 'three';
import { frameAt, dominantClip, loadReport } from './collision-data.js';

const QC = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), -Math.PI / 2); // Z up -> Y up
// Parents before children (world orientations are applied top-down).
const ORDER = ['pelvis', 'torso_lower', 'torso_upper', 'head', 'upperarm_l', 'lowerarm_l', 'hand_l', 'upperarm_r', 'lowerarm_r',
  'hand_r', 'thigh_l', 'calf_l', 'foot_l', 'thigh_r', 'calf_r', 'foot_r'];

export function createReactLink({ url, onStatus }) {
  let enabled = false, report = null, loading = null, root = null;
  const bones = new Map(), rest = new Map();
  const _a = new THREE.Quaternion(), _b = new THREE.Quaternion(), _pw = new THREE.Quaternion(), _rw = new THREE.Quaternion();

  function setModel(gltfRoot) { root = gltfRoot; bones.clear(); gltfRoot.traverse(o => { if (o.isBone) bones.set(o.name, o); }); }

  function status(text) { onStatus?.(text ?? (!enabled ? 'self-collision response' : report ? `${report.glb}` : 'loading…'), enabled); }

  function load() {
    if (report || loading) return loading;
    loading = loadReport(url)
      .then(j => {
        report = j;
        for (const [body, r] of Object.entries(j.rest)) { const [w, x, y, z] = r.quat_rest_wxyz; rest.set(body, { bone: r.bone, q: new THREE.Quaternion(x, y, z, w) }); }
        status();
      })
      .catch(e => { report = null; loading = null; status(`no REACT report (${e.message}): run .venv/bin/python physics/clip_react.py`); });
    return loading;
  }

  function setEnabled(on) { enabled = !!on; if (enabled) load(); status(); }

  /** Interpolated reacted body quaternion (report frame, Z up) at clip time t. */
  function bodyQuat(clip, t, body, out) {
    const n = clip.frames.length, x = Math.max(0, Math.min(n - 1, (clip.loop !== false && t > clip.duration ? t % clip.duration : t) * clip.fps));
    const i = Math.min(n - 2, Math.floor(x)), u = Math.max(0, Math.min(1, x - i));
    const [, , , w0, x0, y0, z0] = clip.frames[i].bodies[body], [, , , w1, x1, y1, z1] = clip.frames[i + 1].bodies[body];
    return out.set(x0, y0, z0, w0).slerp(_b.set(x1, y1, z1, w1), u);
  }

  /**
   * After the mixer: override the lead bones with the reacted pose. Returns the frame used (with
   * its "dev" per body) or null when REACT is off / no data for the dominant clip.
   */
  function apply(snapshot) {
    if (!enabled || !report || !root || !snapshot) return null;
    const dom = dominantClip(snapshot), clip = dom && report.clips[dom.name];
    if (!clip || clip.frames.length < 2) { status(dom ? `${dom.name}: not in REACT report (${report.glb}) - run physics/clip_react.py <this GLB>` : 'no clip'); return null; }
    root.updateMatrixWorld(true);
    root.getWorldQuaternion(_rw);
    for (const body of ORDER) {
      const r = rest.get(body), bone = r && bones.get(r.bone);
      if (!bone) continue;
      // bone world (GLB, Y up) = root * QC * R_body * rest   (inverse of clip_retarget.body_quats)
      bodyQuat(clip, dom.time, body, _a);
      const desired = _a.premultiply(QC).premultiply(_rw).multiply(r.q);
      if (bone.parent) { bone.parent.updateMatrixWorld(true); bone.parent.getWorldQuaternion(_pw); bone.quaternion.copy(_pw.invert().multiply(desired)); }
      else bone.quaternion.copy(desired);
      bone.updateMatrixWorld(true);
    }
    const fr = frameAt(clip, dom.time);
    const top = Object.entries(fr.dev).sort((p, q) => q[1][0] - p[1][0])[0];
    const nc = clip.no_contact ? ` (drives alone ≤ ${clip.no_contact.max_dev_mm.toFixed(0)} mm)` : '';
    status(`${dom.weight < 0.95 ? '≈ ' : ''}${fr.t.toFixed(2)} s · off clip: ${top[0]} ${top[1][0].toFixed(0)} mm / ${top[1][1].toFixed(0)}°${nc}`);
    return fr;
  }

  return { setModel, setEnabled, apply, load, get enabled() { return enabled; }, get report() { return report; } };
}
