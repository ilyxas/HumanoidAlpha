// COLLISIONS toggle for the Animation mode (physics-free). Shows where the playing clip makes
// Xandra's body pass through itself or the floor, from the precomputed report of
// physics/clip_collisions.py (MuJoCo collision hulls placed on the clip's bones, mj_collision
// only, no physics step). The animation itself is untouched.
//
// Per hit: two points, one on each body part (the deepest point of that part inside the other),
// drawn as spheres joined by a line, plus a row in the panel with both points as (x, y, z).
// Panel coordinates are the report's: metres, Blender/MuJoCo frame (Z up), clip-local.
// The frame shown is the one of the clip with the largest weight; during a crossfade the body is
// a blend of clips, so the overlay is marked "≈".
// "hulls" draws the convex hulls that were actually tested (one per body part, placed on the
// clip's bones for that frame): the hits come from these shapes touching, not from the skin.
import * as THREE from 'three';
import { dominantClip, frameAt, glbFromReport, loadReport } from './collision-data.js';

const KIND_COLOR = { self: 0xff3b30, joint: 0xffb020, floor: 0x3aa0ff };

const fmt = p => `(${p.map(v => (v >= 0 ? '+' : '') + v.toFixed(3)).join(', ')})`;

export function createCollisionOverlay({ scene, container, url, onStatus }) {
  let enabled = false, base = null, loading = null, root = null, showJoint = true, showHulls = false, lastKey = '';
  let alt = null;     // another report in the same frame format (REACT) shown instead of the clip's
  let hullRoot = null; // Z-up frame of the report under the GLB root; one child per body
  const hullBodies = new Map();
  const group = new THREE.Group(); group.name = 'CollisionOverlay'; group.visible = false; group.renderOrder = 999;
  scene.add(group);
  const sphereGeo = new THREE.SphereGeometry(0.016, 12, 8);
  const pool = [];
  function marker(i) {
    if (pool[i]) return pool[i];
    const mat = k => new THREE.MeshBasicMaterial({ color: k, depthTest: false, transparent: true, opacity: 0.95 });
    const a = new THREE.Mesh(sphereGeo, mat(0xffffff)), b = new THREE.Mesh(sphereGeo, mat(0xffffff));
    const line = new THREE.Line(new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()]),
      new THREE.LineBasicMaterial({ color: 0xffffff, depthTest: false, transparent: true }));
    for (const o of [a, b, line]) { o.renderOrder = 999; group.add(o); }
    return (pool[i] = { a, b, line });
  }

  const panel = document.createElement('div');
  panel.className = 'coll-panel'; panel.hidden = true;
  panel.innerHTML = '<div class="cp-head"><span class="cp-title"></span><span><label><input type="checkbox" data-opt="hulls"> hulls</label> <label><input type="checkbox" data-opt="joint" checked> joint</label></span></div>'
    + '<canvas class="cp-strip" width="300" height="34"></canvas><div class="cp-rows"></div>';
  container.append(panel);
  const title = panel.querySelector('.cp-title'), rowsEl = panel.querySelector('.cp-rows'), strip = panel.querySelector('.cp-strip');
  panel.querySelector('[data-opt="joint"]').onchange = e => { showJoint = e.target.checked; lastKey = ''; };
  panel.querySelector('[data-opt="hulls"]').onchange = e => setHulls(e.target.checked);

  function setModel(gltfRoot) { root = gltfRoot; }

  function buildHulls() {
    const report = alt || base;
    if (hullRoot || !report?.hulls || !root) return;
    hullRoot = new THREE.Group(); hullRoot.name = 'CollisionHulls';
    hullRoot.rotation.x = -Math.PI / 2; // report frame (Z up) -> GLB (Y up), same as glbFromReport
    const fill = new THREE.MeshBasicMaterial({ color: 0x40e0d0, transparent: true, opacity: 0.18, depthWrite: false, side: THREE.DoubleSide });
    const edge = new THREE.LineBasicMaterial({ color: 0x40e0d0, transparent: true, opacity: 0.7 });
    for (const h of report.hulls) {
      let body = hullBodies.get(h.body);
      if (!body) { body = new THREE.Group(); body.name = `hull_${h.body}`; hullBodies.set(h.body, body); hullRoot.add(body); }
      const geo = new THREE.BufferGeometry();
      geo.setAttribute('position', new THREE.Float32BufferAttribute(h.verts.flat(), 3));
      geo.setIndex(h.faces.flat());
      body.add(new THREE.Mesh(geo, fill), new THREE.LineSegments(new THREE.EdgesGeometry(geo, 20), edge));
    }
    root.add(hullRoot);
    hullRoot.visible = false;
  }

  function setHulls(on) { showHulls = !!on; buildHulls(); if (hullRoot) hullRoot.visible = enabled && showHulls; }

  async function load() {
    if (base || loading) return loading;
    loading = loadReport(url)
      .then(j => { base = j; status(); })
      .catch(e => { base = null; loading = null; status(`no report (${e.message}): run .venv/bin/python physics/clip_collisions.py`); });
    return loading;
  }

  function status(extra) {
    const text = !enabled ? 'precomputed clip report' : extra || (base ? `${base.glb} · ${Object.keys(base.clips).length} clips` : 'loading…');
    onStatus?.(text, enabled);
  }

  function setEnabled(on) {
    enabled = !!on; panel.hidden = !enabled; group.visible = enabled; lastKey = '';
    if (hullRoot) hullRoot.visible = enabled && showHulls;
    if (enabled) load();
    status();
  }

  function drawStrip(clip, time) {
    const ctx = strip.getContext('2d'), w = strip.width, h = strip.height;
    ctx.clearRect(0, 0, w, h);
    ctx.fillStyle = 'rgba(255,255,255,.08)'; ctx.fillRect(0, 0, w, h);
    const evs = clip.events.filter(e => showJoint || e.kind !== 'joint');
    const lane = Math.max(3, Math.min(8, (h - 4) / Math.max(1, evs.length)));
    evs.forEach((e, i) => {
      ctx.fillStyle = '#' + KIND_COLOR[e.kind].toString(16).padStart(6, '0');
      const x0 = (e.start / clip.duration) * w, x1 = Math.max(x0 + 2, ((e.end + 1 / clip.fps) / clip.duration) * w);
      ctx.fillRect(x0, 2 + (i * lane) % (h - 4), x1 - x0, lane - 1);
    });
    const t = clip.loop !== false && time > clip.duration ? time % clip.duration : Math.min(time, clip.duration);
    ctx.fillStyle = '#ffffff'; ctx.fillRect((t / clip.duration) * w - 1, 0, 2, h);
  }

  /** Per frame, after the mixer: snapshot = anim.snapshot(); source = REACT report or null. */
  function update(snapshot, source = null) {
    if (source !== alt) { alt = source; lastKey = ''; }
    const report = alt || base;
    if (!enabled || !report || !root || !snapshot) { group.visible = false; if (hullRoot) hullRoot.visible = false; return null; }
    const dom = dominantClip(snapshot), clip = dom && report.clips[dom.name];
    if (!clip) { group.visible = false; if (hullRoot) hullRoot.visible = false; title.textContent = dom ? `${dom.name}: not in ${report.glb} report - run .venv/bin/python physics/clip_collisions.py <this GLB>` : 'no clip'; rowsEl.textContent = ''; return null; }
    const fr = frameAt(clip, dom.time);
    const hits = fr.hits.filter(h => showJoint || h.kind !== 'joint');
    if (showHulls && fr.bodies) {
      buildHulls();
      hullRoot.visible = true;
      for (const [name, [x, y, z, qw, qx, qy, qz]] of Object.entries(fr.bodies)) {
        const b = hullBodies.get(name);
        if (b) { b.position.set(x, y, z); b.quaternion.set(qx, qy, qz, qw); }
      }
    } else if (hullRoot) hullRoot.visible = false;
    root.updateMatrixWorld(true);
    group.visible = true;
    hits.forEach((h, i) => {
      const m = marker(i), pa = root.localToWorld(new THREE.Vector3(...glbFromReport(h.pa))), pb = root.localToWorld(new THREE.Vector3(...glbFromReport(h.pb)));
      m.a.position.copy(pa); m.b.position.copy(pb);
      m.a.material.color.setHex(KIND_COLOR[h.kind]); m.b.material.color.setHex(0xffffff); m.line.material.color.setHex(KIND_COLOR[h.kind]);
      m.line.geometry.setFromPoints([pa, pb]);
      for (const o of [m.a, m.b, m.line]) o.visible = true;
    });
    for (let i = hits.length; i < pool.length; i++) for (const o of Object.values(pool[i])) o.visible = false;
    const approx = dom.weight < 0.95 ? '≈ ' : '';
    title.textContent = `${alt ? 'REACT ' : ''}${approx}${dom.name} ${fr.t.toFixed(2)} s · ${hits.length} hit${hits.length === 1 ? '' : 's'}`;
    drawStrip(clip, dom.time);
    const key = `${dom.name}|${fr.t}|${showJoint}`;
    if (key !== lastKey) {
      lastKey = key;
      rowsEl.replaceChildren(...hits.map(h => {
        const row = document.createElement('div');
        row.className = `cp-row cp-${h.kind}`;
        const force = h.force_n != null ? ` · ${h.force_n.toFixed(0)} N` : '';
        row.innerHTML = `<b>${h.a} ↔ ${h.b}</b> <span>${h.depth_mm.toFixed(1)} mm · ${h.kind}${force}</span>`
          + `<div>a ${fmt(h.pa)}</div><div>b ${fmt(h.pb)}</div>`;
        return row;
      }));
    }
    return hits;
  }

  return { setModel, setEnabled, setHulls, update, load, get enabled() { return enabled; }, get report() { return base; } };
}
