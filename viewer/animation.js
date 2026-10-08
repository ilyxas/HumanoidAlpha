// Animation mode: same screen as Observation (camera, presets, LOCK, clock,
// Screenshot, on-screen sticks), but NO physics and NO WebSocket.
// EXP002.1-B: Xandra is an animation-driven character. Keyboard (WASD, Shift,
// Caps Lock), the LEFT stick (touch/mouse/gamepad) and the legacy RIGHT stick
// all feed ONE movement controller (human/locomotion.js) that moves her root
// through the world with delta time; the animation controller
// (animation/anim-controller.js) crossfades the in-place clips with playback
// rate = ground speed / manifest natural speed. The Animation Debugger and the
// facial morph inspector live in animation/debugger.js.
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { createProfiler } from './observation-perf.js';
import { readGamepad } from './human/normalize.js';
import { mountSticks } from './human/sticks-ui.js';
import { createWalkSession } from './human/walk-control.js';
import { DEFAULT_SPEEDS, createMovementController, keyboardInput, selectLocomotion, stickInput } from './human/locomotion.js';
import { EXPECTED_CLIPS, createAnimationController } from './animation/anim-controller.js';
import { mountDebugger } from './animation/debugger.js';

const MODEL_URL = '../assets/Xandra_Animated.glb';
const MANIFEST_URL = '../assets/Xandra_Animated.manifest.json';

const params = new URLSearchParams(location.search);
const perf = createProfiler(params.get('profile') === '1');
const canvas = document.querySelector('#scene'), overlay = document.querySelector('#overlay');
const context = overlay.getContext('2d');
const renderer = new THREE.WebGLRenderer({ canvas, antialias: true, preserveDrawingBuffer: true });
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
renderer.outputColorSpace = THREE.SRGBColorSpace;
renderer.toneMapping = THREE.ACESFilmicToneMapping;
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
const scene = new THREE.Scene(); scene.background = new THREE.Color(0x66717e);
const camera = new THREE.PerspectiveCamera(40, 2, .05, 80);
const controls = new OrbitControls(camera, canvas);
controls.enableDamping = false;
scene.add(new THREE.HemisphereLight(0xffffff, 0x66717e, 1.4));
const light = new THREE.DirectionalLight(0xffffff, 2); light.castShadow = true;
light.shadow.mapSize.set(2048, 2048);
Object.assign(light.shadow.camera, { left: -5, right: 5, top: 5, bottom: -5, near: .1, far: 20 });
light.shadow.normalBias = .015;
const LIGHT_OFFSET = new THREE.Vector3(3, 6, 4);
scene.add(light, light.target);
// Same floor material and 1 m grid as Observation, larger so she can walk somewhere.
const FLOOR_SIZE = 200;
const floor = new THREE.Mesh(new THREE.PlaneGeometry(FLOOR_SIZE, FLOOR_SIZE), new THREE.MeshStandardMaterial({ color: 0xf3f2ed, roughness: 1, metalness: 0 }));
floor.rotation.x = -Math.PI / 2; floor.receiveShadow = true; scene.add(floor);
const grid = new THREE.GridHelper(FLOOR_SIZE, FLOOR_SIZE, 0xd2d3d3, 0xd2d3d3); grid.position.y = .001; scene.add(grid);

// The character root that moves through the world. The GLB scene sits inside
// it untouched (faces +Z at yaw 0, the GLB convention).
const character = new THREE.Group(); character.name = 'XandraCharacter'; scene.add(character);

// Camera presets: same poses as Observation, expressed relative to Xandra so
// they stay useful while she walks. "camera follow" translates the orbit
// target and camera together (user orbit/zoom preserved); off = fixed camera.
const TARGET_OFFSET = new THREE.Vector3(0, .7, 0);
const presets = { FRONT: [0, 1.6, 5.5], LEFT: [5.5, 1.6, 0], BACK: [0, 1.6, -5.5], RIGHT: [-5.5, 1.6, 0], TOP: [0, 6.2, 0], RESET: [3.6, 2.5, 4.6] };
const errEl = document.getElementById('err');
let locked = false, follow = true;
const followPos = new THREE.Vector3();
function preset(name) {
  const base = follow ? followPos : new THREE.Vector3();
  controls.target.copy(base).add(TARGET_OFFSET); camera.position.copy(base).add(new THREE.Vector3(...presets[name]));
  camera.up.set(0, name === 'TOP' ? 0 : 1, name === 'TOP' ? -1 : 0);
  camera.lookAt(controls.target); controls.update(); renderScene();
}
for (const button of document.querySelectorAll('[data-view]')) button.onclick = () => preset(button.dataset.view);
const lockButton = document.querySelector('#lock');
lockButton.onclick = () => { locked = !locked; controls.enabled = !locked; lockButton.textContent = locked ? 'UNLOCK' : 'LOCK'; lockButton.setAttribute('aria-label', locked ? 'Unlock camera' : 'Lock camera'); lockButton.setAttribute('aria-pressed', String(locked)); drawOverlay(); };
canvas.addEventListener('contextmenu', e => e.preventDefault());

// ---- Input: sticks (same widgets as Observation) + keyboard -> one controller ----
const session = createWalkSession();
const human = document.getElementById('human');
const sticksUi = mountSticks(human, session);
// Authority has no meaning without actuators; keep the widget for layout parity.
for (const b of document.querySelectorAll('#human [aria-label$="authority"]')) b.disabled = true;
human.addEventListener('pointerdown', e => session.setPointerType(e.pointerType), true);
let lastSample = session.sample();

const keys = { w: false, a: false, s: false, d: false };
const KEY_OF = { KeyW: 'w', KeyA: 'a', KeyS: 's', KeyD: 'd' };
let shiftHeld = false, capsLock = false;
const isMac = /Mac|iPhone|iPad/.test(navigator.platform || '');
function typingTarget(e) { const t = e.target; return t && (t.tagName === 'SELECT' || t.tagName === 'TEXTAREA' || (t.tagName === 'INPUT' && /^(text|search|number)$/.test(t.type))); }
function onKey(e) {
  const down = e.type === 'keydown';
  if (e.code === 'CapsLock') {
    // macOS Chrome reports CapsLock as keydown = on, keyup = off; elsewhere each press is a keydown.
    if (isMac) capsLock = down; else if (down && !e.repeat) capsLock = !capsLock;
    return;
  }
  if (e.key === 'Shift') { shiftHeld = down; return; }
  shiftHeld = e.shiftKey;
  if (down && e.getModifierState?.('CapsLock') && !isMac) capsLock = true; // page opened with Caps on
  const k = KEY_OF[e.code];
  if (!k || (down && (typingTarget(e) || e.ctrlKey || e.metaKey || e.altKey))) return;
  keys[k] = down;
  if (down) e.preventDefault();
}
addEventListener('keydown', onKey); addEventListener('keyup', onKey);
addEventListener('blur', () => { for (const k in keys) keys[k] = false; shiftHeld = false; });

/** All sources -> one MoveInput. Priority: left stick, right stick (legacy forward walk), keyboard. */
function gatherInput(sample) {
  const runMod = shiftHeld || capsLock;
  let input = sample.move;
  if (!input && sample.walk.active) input = stickInput(0, sample.sticks.right.vertical, { gamma: sample.expo, source: sample.walkSource });
  if (input && runMod) input = { ...input, gait: 1 };
  if (!input) input = keyboardInput(keys, { shift: shiftHeld, capsLock });
  return input;
}

// Ground-plane reference "forward" for input: the camera's view direction on the floor
// (TOP view looks straight down, so its screen-up vector is used instead).
const _dir = new THREE.Vector3();
function inputForward() {
  camera.getWorldDirection(_dir); _dir.y = 0;
  if (_dir.lengthSq() < 1e-4) { _dir.copy(camera.up).applyQuaternion(camera.quaternion); _dir.y = 0; }
  if (_dir.lengthSq() < 1e-8) _dir.set(0, 0, -1);
  _dir.normalize();
  return { x: _dir.x, z: _dir.z };
}

// ---- Model, manifest, controllers ----
let anim = null, manifest = null, clips = [], gltfRoot = null, locoState = 'Idle';
const movement = createMovementController();
let morph = { names: [], meshes: [], meshName: null };
const feet = {};
let measuredSpeed = 0, simTime = 0;
const lastPos = new THREE.Vector3();
let recording = null;

function collectMorphs(root) {
  const meshes = [];
  root.traverse(o => { if (o.isMesh && o.morphTargetDictionary && Object.keys(o.morphTargetDictionary).length) meshes.push(o); });
  if (!meshes.length) return { names: [], meshes: [], meshName: null };
  const dict = meshes[0].morphTargetDictionary;
  const names = Object.keys(dict).sort((a, b) => dict[a] - dict[b]);
  const owner = meshes[0].parent && meshes[0].parent.isGroup && !meshes[0].parent.isScene ? meshes[0].parent.name : meshes[0].name;
  return { names, meshes, meshName: owner };
}
function getMorph(name) { const m = morph.meshes[0]; return m ? m.morphTargetInfluences[m.morphTargetDictionary[name]] || 0 : 0; }
function setMorph(name, w) {
  for (const m of morph.meshes) { const i = m.morphTargetDictionary[name]; if (i != null) m.morphTargetInfluences[i] = w; }
}
function resetFace() { for (const m of morph.meshes) m.morphTargetInfluences.fill(0); }

const debug = mountDebugger(document.body, {
  onPause() { if (anim) anim.paused = !anim.paused; },
  onMode() { if (anim) anim.setMode(anim.mode === 'auto' ? 'manual' : 'auto'); },
  onSpeed(v) { if (anim) anim.speed = v; },
  onPreview(name) { if (anim) anim.preview(name); },
  onFollow(v) { follow = v; if (v) followPos.copy(character.position); },
  onRunLock(v) { capsLock = v; },
  onResetFace: resetFace,
}, { open: innerWidth >= 900 });

const loadStart = performance.now(); perf.event('model-load-start');
const manifestReady = fetch(MANIFEST_URL, { cache: 'no-cache' }).then(r => (r.ok ? r.json() : null)).catch(() => null);
new GLTFLoader().load(MODEL_URL, async gltf => {
  manifest = await manifestReady;
  gltfRoot = gltf.scene;
  gltfRoot.traverse(o => { if (o.isMesh) { o.castShadow = true; o.receiveShadow = true; } });
  character.add(gltfRoot);
  clips = gltf.animations;
  const speeds = { ...DEFAULT_SPEEDS };
  for (const name of Object.keys(speeds)) {
    const v = manifest?.clips?.[name]?.natural_ground_speed_mps;
    if (Number.isFinite(v) && v > 0) speeds[name] = v;
  }
  movement.setSpeeds(speeds);
  anim = createAnimationController({ root: gltfRoot, clips, manifest });
  if (!manifest) errEl.textContent = 'Manifest not found: using built-in speeds; one-shot loop flags assumed.';
  for (const n of ['foot_l', 'foot_r', 'ball_l', 'ball_r']) feet[n] = gltfRoot.getObjectByName(n) || null;
  const footPhases = anim.analyzeFootPhases(feet.foot_l);
  morph = collectMorphs(gltfRoot);
  const order = manifest?.global_info?.clip_names || EXPECTED_CLIPS;
  const listed = [...clips].sort((a, b) => ((order.indexOf(a.name) + 1) || 99) - ((order.indexOf(b.name) + 1) || 99));
  const info = listed.map(c => { const e = anim.entries.get(c.name); return { name: c.name, duration: c.duration, loop: e.loop, natural: e.natural }; });
  debug.setClips(info);
  debug.setMorphs(morph.names, getMorph, setMorph, morph.meshName);
  session.setMoveLabel('move'); session.setWalkLabel('walk');
  console.info('[animation] clips', JSON.stringify(info.map(c => ({ name: c.name, duration: +c.duration.toFixed(3), loop: c.loop }))),
    'speeds', JSON.stringify(speeds), 'footPhases', JSON.stringify(footPhases), 'morphs', morph.names.length);
  if (anim.warnings.length) console.warn('[animation]', anim.warnings.join('; '));
  perf.instrument(gltfRoot); perf.event('model-load-complete', { elapsedMs: performance.now() - loadStart });
}, undefined, error => { perf.event('model-load-error'); errEl.textContent = 'Model load failed: ' + (error?.message || error); });

const _feetTmp = new THREE.Vector3();
function feetWorld() {
  const out = {};
  for (const [n, b] of Object.entries(feet)) if (b) { b.getWorldPosition(_feetTmp); out[n] = [_feetTmp.x, _feetTmp.y, _feetTmp.z]; }
  return out;
}

let lastInput = { x: 0, y: 0, gait: 0, source: 'none' };
/** Per frame: input -> movement controller -> character transform -> animation controller. */
function update(dt) {
  session.setGamepad(readGamepad());
  lastSample = session.sample();
  sticksUi.paint(lastSample);
  if (!anim) return;
  const paused = anim.paused;
  const simDt = paused ? 0 : dt * anim.speed;
  const input = anim.mode === 'auto' ? gatherInput(lastSample) : null;
  const mv = movement.update(simDt, input, inputForward());
  lastInput = mv.input;
  lastPos.copy(character.position);
  character.position.set(mv.position.x, 0, mv.position.z);
  character.rotation.y = mv.yaw;
  if (simDt > 0) {
    const inst = lastPos.distanceTo(character.position) / simDt;
    measuredSpeed += (inst - measuredSpeed) * Math.min(1, simDt * 10);
    simTime += simDt;
  }
  if (anim.mode === 'auto') {
    const sel = selectLocomotion(locoState, mv.speed, movement.speeds, movement.config, anim.available);
    locoState = sel.state;
    anim.setLocomotion({ ...sel, speedAbs: Math.abs(mv.speed) });
  } else locoState = 'Idle';
  anim.update(dt);
  if (recording && simDt > 0) {
    character.updateMatrixWorld(true);
    const snap = anim.snapshot();
    recording.push({ t: +simTime.toFixed(4), dt: simDt, x: mv.position.x, z: mv.position.z, yaw: mv.yaw, speed: mv.speed, measured: measuredSpeed, state: anim.state,
      clips: snap.clips.filter(c => c.weight > 0.001).map(c => ({ name: c.name, w: +c.weight.toFixed(3), ts: +c.timeScale.toFixed(4), time: +c.time.toFixed(4) })), feet: feetWorld() });
    if (recording.length > 20000) recording.shift();
  }
}

const _prev = new THREE.Vector3();
function updateFollow(dt) {
  if (!follow) return;
  _prev.copy(followPos);
  followPos.lerp(character.position, 1 - Math.exp(-6 * dt));
  const delta = _prev.subVectors(followPos, _prev);
  controls.target.add(delta); camera.position.add(delta);
  light.position.copy(followPos).add(LIGHT_OFFSET); light.target.position.copy(followPos);
}
light.position.copy(LIGHT_OFFSET); light.target.position.set(0, 0, 0);
preset('RESET');

// UI and screenshots share this exact overlay. Native buttons supply input/accessibility.
let rects = [];
const clock = document.querySelector('#clock');
function clockText() { return new Date().toLocaleTimeString('en-GB', { hour12: false, hour: '2-digit', minute: '2-digit', second: '2-digit' }); }
function drawOverlay() {
  const w = innerWidth, h = innerHeight, dpr = renderer.getPixelRatio();
  context.setTransform(dpr, 0, 0, dpr, 0, 0); context.clearRect(0, 0, w, h);
  context.fillStyle = 'rgba(24,32,43,.88)'; context.beginPath(); context.roundRect(16, 16, 100, 36, 8); context.fill();
  context.font = '500 16px monospace'; context.textAlign = 'center'; context.textBaseline = 'middle'; context.fillStyle = '#f5f7fa'; context.fillText(clock.textContent, 66, 34);
  for (const { button, x, y, width, height } of rects) {
    const active = button === lockButton && locked;
    context.fillStyle = active ? '#d5eee5' : 'rgba(24,32,43,.90)'; context.beginPath(); context.roundRect(x, y, width, height, 7); context.fill();
    if (document.activeElement === button) { context.strokeStyle = '#ffffff'; context.lineWidth = 2; context.stroke(); }
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
for (const button of document.querySelectorAll('#camera button')) { button.addEventListener('focus', drawOverlay); button.addEventListener('blur', drawOverlay); }
function tickClock() { clock.textContent = clockText(); drawOverlay(); }
tickClock(); setInterval(tickClock, 250); addEventListener('resize', resize); resize();
document.querySelector('#shot').onclick = () => {
  const shot = document.createElement('canvas'); shot.width = canvas.width; shot.height = canvas.height;
  const ctx = shot.getContext('2d'); ctx.drawImage(canvas, 0, 0); ctx.drawImage(overlay, 0, 0, shot.width, shot.height);
  const now = new Date(), pad = n => String(n).padStart(2, '0');
  const stamp = `${now.getFullYear()}-${pad(now.getMonth() + 1)}-${pad(now.getDate())}_${pad(now.getHours())}-${pad(now.getMinutes())}-${pad(now.getSeconds())}`;
  const link = document.createElement('a'); link.download = `animation_${stamp}.png`; link.href = shot.toDataURL('image/png'); document.body.append(link); link.click(); link.remove();
};
function renderScene() { const start = performance.now(); renderer.render(scene, camera); perf.sample('rendererCpuMs', performance.now() - start); }
const clockDelta = new THREE.Clock();
function liveData() {
  const mv = movement.snapshot();
  return { anim: anim.snapshot(), move: mv, input: lastInput, measuredSpeed, run: { shift: shiftHeld, capsLock }, simTime };
}
function animate(t) {
  requestAnimationFrame(animate); perf.frame(t); const start = performance.now();
  const dt = Math.min(clockDelta.getDelta(), 0.1);
  update(dt);
  updateFollow(dt);
  if (anim && debug.open) debug.update(liveData());
  controls.update(); renderScene(); perf.sample('frameWorkCpuMs', performance.now() - start);
}
requestAnimationFrame(animate);

// Read-only hook (plus recorder/morph helpers) for automated checks in headless tests.
window.__animation = {
  get ready() { return !!anim; },
  get clips() { return clips.map(c => ({ name: c.name, duration: c.duration, loop: anim?.entries.get(c.name)?.loop ?? null })); },
  get morphs() { return { names: [...morph.names], meshes: morph.meshes.length, meshName: morph.meshName }; },
  getMorph, setMorph,
  state() { return anim ? { ...liveData(), locoState, camera: { target: controls.target.toArray(), position: camera.position.toArray() }, follow } : null; },
  feet() { character.updateMatrixWorld(true); return feetWorld(); },
  /** Test helper: place the orbit camera (world coords); follow is switched off so it stays put. */
  view(position, target) { follow = false; controls.target.set(...target); camera.position.set(...position); camera.up.set(0, 1, 0); camera.lookAt(controls.target); controls.update(); return true; },
  record(on = true) { recording = on ? [] : recording; return true; },
  takeRecording() { const r = recording || []; recording = null; return r; },
};
