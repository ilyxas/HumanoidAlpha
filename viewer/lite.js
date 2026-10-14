// Xandra-Lite page (EXPERIMENTAL, ./START.command lite). Renders the MuJoCo primitives of the
// reduced body exactly as the runtime describes them (inventory.geoms), moves them from the
// streamed body poses, and sends commands: ActionFrame (walk/crouch), assist on/off, reset, and the
// joystick torques for the 11 motors through the same stick modules as Harness (viewer/human/).
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { createHumanControls } from './human/attach.js';

const NU = 11;
const params = new URLSearchParams(location.search);
const canvas = document.querySelector('#scene'), overlay = document.querySelector('#overlay');
const context = overlay.getContext('2d');
const errEl = document.getElementById('err');
const statusEl = document.getElementById('liteStatus');
const renderer = new THREE.WebGLRenderer({ canvas, antialias: true, preserveDrawingBuffer: true });
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
renderer.outputColorSpace = THREE.SRGBColorSpace;
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
const scene = new THREE.Scene(); scene.background = new THREE.Color(0x66717e);
const camera = new THREE.PerspectiveCamera(40, 2, .05, 80);
const controls = new OrbitControls(camera, canvas);
controls.enableDamping = false;
scene.add(new THREE.HemisphereLight(0xffffff, 0x66717e, 1.4));
const light = new THREE.DirectionalLight(0xffffff, 2); light.castShadow = true;
light.shadow.mapSize.set(2048, 2048);
Object.assign(light.shadow.camera, { left: -4, right: 4, top: 4, bottom: -4, near: .1, far: 20 });
scene.add(light, light.target);
const floor = new THREE.Mesh(new THREE.PlaneGeometry(200, 200), new THREE.MeshStandardMaterial({ color: 0xf3f2ed, roughness: 1 }));
floor.rotation.x = -Math.PI / 2; floor.receiveShadow = true; scene.add(floor);
const grid = new THREE.GridHelper(200, 200, 0xd2d3d3, 0xd2d3d3); grid.position.y = .001; scene.add(grid);

// MuJoCo world (Z up, she faces +X) inside a Y-up three.js scene: one fixed rotation, nothing else.
const world = new THREE.Group(); world.rotation.x = -Math.PI / 2; scene.add(world);
const bodies = [];
let torsoId = 1;

function geomMesh(g) {
  const [a, b, c] = g.size;
  let geo;
  if (g.type === 'sphere') geo = new THREE.SphereGeometry(a, 24, 16);
  else if (g.type === 'capsule') { geo = new THREE.CapsuleGeometry(a, 2 * b, 8, 16); geo.rotateX(Math.PI / 2); }
  else if (g.type === 'cylinder') { geo = new THREE.CylinderGeometry(a, a, 2 * b, 20); geo.rotateX(Math.PI / 2); }
  else if (g.type === 'box') geo = new THREE.BoxGeometry(2 * a, 2 * b, 2 * c);
  else return null; // plane: the floor above
  const color = new THREE.Color(g.rgba[0], g.rgba[1], g.rgba[2]);
  const mesh = new THREE.Mesh(geo, new THREE.MeshStandardMaterial({ color, roughness: .7, metalness: 0 }));
  mesh.position.set(...g.pos);
  mesh.quaternion.set(g.quat[1], g.quat[2], g.quat[3], g.quat[0]);
  mesh.castShadow = true; mesh.receiveShadow = true;
  return mesh;
}

function buildModel(inv) {
  for (const b of bodies) world.remove(b);
  bodies.length = 0;
  for (const b of inv.bodies) {
    const grp = new THREE.Group(); grp.name = b.name; bodies.push(grp); world.add(grp);
    if (b.name === 'torso') torsoId = b.id;
  }
  for (const g of inv.geoms) {
    if (g.body === 0 && g.type === 'plane') continue;
    const mesh = geomMesh(g);
    if (mesh) bodies[g.body].add(mesh);
  }
}

function applyPoses(poses) {
  for (let i = 0; i < poses.length && i < bodies.length; i++) {
    const p = poses[i];
    bodies[i].position.set(p[0], p[1], p[2]);
    bodies[i].quaternion.set(p[4], p[5], p[6], p[3]);
  }
}

// ---- camera: presets relative to her position (she faces +X in MuJoCo = +X here; her left = -Z)
const TARGET_Y = .8;
const presets = { FRONT: [5.5, 1.6, 0], LEFT: [0, 1.6, -5.5], BACK: [-5.5, 1.6, 0], RIGHT: [0, 1.6, 5.5], TOP: [0, 6.2, 0], RESET: [3.6, 2.5, 4.6] };
let locked = false;
const follow = new THREE.Vector3();
function torsoWorld(out) {
  if (!bodies[torsoId]) return out.set(0, 0, 0);
  bodies[torsoId].getWorldPosition(out); return out;
}
function preset(name) {
  torsoWorld(follow); follow.y = 0;
  controls.target.set(follow.x, TARGET_Y, follow.z);
  camera.position.set(follow.x + presets[name][0], presets[name][1], follow.z + presets[name][2]);
  camera.up.set(0, name === 'TOP' ? 0 : 1, name === 'TOP' ? -1 : 0);
  camera.lookAt(controls.target); controls.update();
}
preset('RESET');
for (const button of document.querySelectorAll('[data-view]')) button.onclick = () => preset(button.dataset.view);
const lockButton = document.querySelector('#lock');
lockButton.onclick = () => { locked = !locked; controls.enabled = !locked; lockButton.textContent = locked ? 'UNLOCK' : 'LOCK'; lockButton.setAttribute('aria-pressed', String(locked)); drawOverlay(); };
canvas.addEventListener('contextmenu', e => e.preventDefault());
const _p = new THREE.Vector3(), _prev = new THREE.Vector3();
function updateFollow(dt) {
  torsoWorld(_p); _p.y = 0;
  _prev.copy(follow);
  follow.lerp(_p, 1 - Math.exp(-6 * dt));
  const delta = _prev.subVectors(follow, _prev);
  controls.target.add(delta); camera.position.add(delta);
  light.position.set(follow.x + 3, 6, follow.z + 4); light.target.position.copy(follow);
}

// ---- commands
let socket = null, reconnect = null, canControl = false;
const human = createHumanControls(document, { nu: NU, initial: ['shoulder', 'hip_pitch_l'] });
const action = { move: 0, crouch: false };
let assist = true;
function send(msg) { if (socket && socket.readyState === 1) socket.send(JSON.stringify(msg)); }
function sendAction() { send({ op: 'action', move: action.move, crouch: action.crouch }); paintButtons(); }
const btn = { walk: document.getElementById('walk'), crouch: document.getElementById('crouch'), assist: document.getElementById('assist'), reset: document.getElementById('reset') };
function paintButtons() {
  btn.walk.setAttribute('aria-pressed', String(action.move > 0));
  btn.crouch.setAttribute('aria-pressed', String(action.crouch));
  btn.assist.setAttribute('aria-pressed', String(assist));
  btn.assist.textContent = assist ? 'ASSIST ON' : 'ASSIST OFF';
}
btn.walk.onclick = () => { action.move = action.move > 0 ? 0 : 1; sendAction(); btn.walk.blur(); };
btn.crouch.onclick = () => { action.crouch = !action.crouch; sendAction(); btn.crouch.blur(); };
btn.assist.onclick = () => { assist = !assist; send({ op: 'assist', value: assist }); paintButtons(); btn.assist.blur(); };
btn.reset.onclick = () => { action.move = 0; action.crouch = false; send({ op: 'reset' }); sendAction(); btn.reset.blur(); };

function typing(e) { const t = e.target; return t && (t.tagName === 'INPUT' || t.tagName === 'TEXTAREA' || t.tagName === 'SELECT'); }
addEventListener('keydown', e => {
  if (typing(e) || e.ctrlKey || e.metaKey || e.altKey) return;
  if (e.code === 'KeyW' && !e.repeat) { action.move = 1; sendAction(); e.preventDefault(); }
  else if (e.code === 'KeyC' && !e.repeat) { action.crouch = !action.crouch; sendAction(); }
  else if (e.code === 'KeyR' && !e.repeat) btn.reset.onclick();
  else if (e.code === 'KeyA' && !e.repeat) btn.assist.onclick();
});
addEventListener('keyup', e => { if (e.code === 'KeyW') { action.move = 0; sendAction(); } });
addEventListener('blur', () => { if (action.move) { action.move = 0; sendAction(); } });

function status(s) {
  const owned = (s.human || []).map((v, i) => [v, i]).filter(([v]) => Math.abs(v) > 1e-6);
  const names = inventory ? inventory.actuators.map(a => a.name) : [];
  statusEl.textContent =
    `t ${s.t.toFixed(1)} s · ${s.skill} · ${s.support}${s.crouch ? ' · crouch' : ''}\n` +
    `assist ${s.assist ? 'ON' : 'OFF'} · feet ${s.feet_N[0].toFixed(0)} / ${s.feet_N[1].toFixed(0)} N · x ${s.com[0].toFixed(2)} m\n` +
    `sticks: ${owned.length ? owned.map(([v, i]) => `${names[i]} ${v.toFixed(1)}`).join(', ') : '—'}` +
    (canControl ? '' : '\nREAD-ONLY: another page controls the runtime');
}

let inventory = null;
function connect() {
  const port = params.get('wsPort') || '8766';
  if (!/^\d{1,5}$/.test(port)) return;
  socket = new WebSocket(`ws://${location.hostname || '127.0.0.1'}:${port}`);
  human.bindSocket(socket);
  socket.onopen = () => { errEl.textContent = ''; };
  socket.onmessage = event => {
    let m; try { m = JSON.parse(event.data); } catch { return; }
    if (m.type === 'inventory') {
      inventory = m.inventory; canControl = !!m.can_control;
      buildModel(inventory); human.onInventory(inventory); preset('RESET'); sendAction();
    } else if (m.type === 'control') { canControl = !!m.can_control; if (canControl) { errEl.textContent = ''; sendAction(); }
    } else if (m.type === 'state') { applyPoses(m.poses); assist = m.assist; paintButtons(); status(m); }
    else if (m.type === 'error') errEl.textContent = m.message;
  };
  socket.onclose = () => { statusEl.textContent = 'runtime disconnected — reconnecting…'; socket = null; reconnect = setTimeout(connect, 1500); };
}
connect();
addEventListener('pagehide', () => { clearTimeout(reconnect); human.stop(); if (socket) { socket.onclose = null; socket.close(); } });

// ---- overlay (clock + camera buttons), same look as Observation
let rects = [];
const clock = document.querySelector('#clock');
function drawOverlay() {
  const w = innerWidth, h = innerHeight, dpr = renderer.getPixelRatio();
  context.setTransform(dpr, 0, 0, dpr, 0, 0); context.clearRect(0, 0, w, h);
  context.fillStyle = 'rgba(24,32,43,.88)'; context.beginPath(); context.roundRect(16, 16, 100, 36, 8); context.fill();
  context.font = '500 16px monospace'; context.textAlign = 'center'; context.textBaseline = 'middle'; context.fillStyle = '#f5f7fa'; context.fillText(clock.textContent, 66, 34);
  for (const { button, x, y, width, height } of rects) {
    const active = button === lockButton && locked;
    context.fillStyle = active ? '#d5eee5' : 'rgba(24,32,43,.90)'; context.beginPath(); context.roundRect(x, y, width, height, 7); context.fill();
    context.fillStyle = active ? '#172d25' : '#f5f7fa'; context.font = '600 12px system-ui'; context.fillText(button.textContent, x + width / 2, y + height / 2);
  }
}
function resize() {
  renderer.setSize(innerWidth, innerHeight, false); camera.aspect = innerWidth / innerHeight; camera.updateProjectionMatrix();
  const dpr = renderer.getPixelRatio(); overlay.width = Math.round(innerWidth * dpr); overlay.height = Math.round(innerHeight * dpr);
  const width = 68, gap = 6, start = innerWidth - 16 - (width * 3 + gap * 2), top = innerWidth < 390 ? 66 : 16;
  rects = [...document.querySelectorAll('#camera button')].map((button, i) => {
    const x = start + (i % 3) * (width + gap), y = top + Math.floor(i / 3) * 42, buttonWidth = i === 7 ? width * 2 + gap : width;
    Object.assign(button.style, { left: `${x}px`, top: `${y}px`, width: `${buttonWidth}px`, height: '36px' });
    return { button, x, y, width: buttonWidth, height: 36 };
  }); drawOverlay();
}
function tickClock() { clock.textContent = new Date().toLocaleTimeString('en-GB', { hour12: false }); drawOverlay(); }
tickClock(); setInterval(tickClock, 250); addEventListener('resize', resize); resize();
document.querySelector('#shot').onclick = () => {
  const shot = document.createElement('canvas'); shot.width = canvas.width; shot.height = canvas.height;
  const ctx = shot.getContext('2d'); ctx.drawImage(canvas, 0, 0); ctx.drawImage(overlay, 0, 0, shot.width, shot.height);
  const link = document.createElement('a'); link.download = `lite_${Date.now()}.png`; link.href = shot.toDataURL('image/png'); document.body.append(link); link.click(); link.remove();
};

const clockDelta = new THREE.Clock();
function animate() {
  requestAnimationFrame(animate);
  const dt = Math.min(clockDelta.getDelta(), .1);
  updateFollow(dt);
  controls.update(); renderer.render(scene, camera);
}
requestAnimationFrame(animate);

// read-only hook for automated checks
window.__lite = { get inventory() { return inventory; }, get bodies() { return bodies.length; }, action, get assist() { return assist; } };
