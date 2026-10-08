// Animation mode: same screen as Observation (camera, presets, LOCK, clock,
// Screenshot, on-screen sticks), but NO physics and NO WebSocket. Xandra is
// driven only by a Three.js AnimationMixer playing clips baked in the GLB.
// RIGHT stick pushed UP plays the walk; the higher, the faster. Release stops it.
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { createProfiler } from './observation-perf.js';
import { readGamepad } from './human/normalize.js';
import { mountSticks } from './human/sticks-ui.js';
import { createWalkSession } from './human/walk-control.js';

const MODEL_URL = '../assets/Xandra_Animated.glb';
const FADE_IN_S = 0.25;   // walk weight 0 -> 1 when the stick leaves the deadzone
const FADE_OUT_S = 0.35;  // walk weight 1 -> 0 (blend back to rest pose) on release

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
const camera = new THREE.PerspectiveCamera(40, 2, .05, 50);
const controls = new OrbitControls(camera, canvas);
controls.enableDamping = false;
scene.add(new THREE.HemisphereLight(0xffffff, 0x66717e, 1.4));
const light = new THREE.DirectionalLight(0xffffff, 2); light.position.set(3, 6, 4); light.castShadow = true;
light.shadow.mapSize.set(2048, 2048);
Object.assign(light.shadow.camera, { left: -5, right: 5, top: 5, bottom: -5, near: .1, far: 20 });
light.shadow.normalBias = .015;
scene.add(light);
const floor = new THREE.Mesh(new THREE.PlaneGeometry(40, 40), new THREE.MeshStandardMaterial({ color: 0xf3f2ed, roughness: 1, metalness: 0 }));
floor.rotation.x = -Math.PI / 2; floor.receiveShadow = true; scene.add(floor);
const grid = new THREE.GridHelper(40, 40, 0xd2d3d3, 0xd2d3d3); grid.position.y = .001; scene.add(grid);
// Fixed world-space camera poses, identical to Observation.
const target = new THREE.Vector3(0, .7, 0);
const presets = { FRONT: [0, 1.6, 5.5], LEFT: [5.5, 1.6, 0], BACK: [0, 1.6, -5.5], RIGHT: [-5.5, 1.6, 0], TOP: [0, 6.2, 0], RESET: [3.6, 2.5, 4.6] };
const statsEl = document.getElementById('stats');
const errEl = document.getElementById('err');
statsEl.style.bottom = '44px'; // keep the walk readout clear of the GAMEPAD status line
let locked = false;
function preset(name) {
  controls.target.copy(target); camera.position.set(...presets[name]);
  camera.up.set(0, name === 'TOP' ? 0 : 1, name === 'TOP' ? -1 : 0);
  camera.lookAt(target); controls.update(); renderScene();
}
preset('RESET');
for (const button of document.querySelectorAll('[data-view]')) button.onclick = () => preset(button.dataset.view);
const lockButton = document.querySelector('#lock');
lockButton.onclick = () => { locked = !locked; controls.enabled = !locked; lockButton.textContent = locked ? 'UNLOCK' : 'LOCK'; lockButton.setAttribute('aria-label', locked ? 'Unlock camera' : 'Lock camera'); lockButton.setAttribute('aria-pressed', String(locked)); drawOverlay(); };
canvas.addEventListener('contextmenu', e => e.preventDefault());

// ---- Sticks: same widgets as Observation, fed by a walk session (no socket). ----
const session = createWalkSession();
const sticksUi = mountSticks(document.getElementById('human'), session);
// Authority has no meaning without actuators; keep the widget for layout parity.
for (const b of document.querySelectorAll('#human [aria-label$="authority"]')) b.disabled = true;
let lastSample = session.sample();

// ---- Model + mixer ----
let root = null, mixer = null, walkAction = null, walkWeight = 0, clips = [];

/** The exported walk carries root motion (pelvis travels forward). Make it in place. */
function makeInPlace(clip) {
  const track = clip.tracks.find(t => /(^|\.)pelvis\.position$/.test(t.name) || t.name === 'pelvis.position');
  if (!track || track.times.length < 2) return 0;
  const n = track.times.length, v = track.values, t0 = track.times[0], t1 = track.times[n - 1];
  const dx = v[(n - 1) * 3] - v[0], dz = v[(n - 1) * 3 + 2] - v[2];
  for (let i = 0; i < n; i++) {
    const f = (track.times[i] - t0) / (t1 - t0);
    v[i * 3] -= dx * f; v[i * 3 + 2] -= dz * f;
  }
  return Math.hypot(dx, dz);
}

const loadStart = performance.now(); perf.event('model-load-start');
new GLTFLoader().load(MODEL_URL, gltf => {
  root = gltf.scene;
  root.traverse(o => { if (o.isMesh) { o.castShadow = true; o.receiveShadow = true; } });
  scene.add(root);
  clips = gltf.animations;
  mixer = new THREE.AnimationMixer(root);
  const walk = clips.find(c => /walk/i.test(c.name)) || clips[0];
  if (walk) {
    const removed = makeInPlace(walk);
    walkAction = mixer.clipAction(walk);
    walkAction.setLoop(THREE.LoopRepeat, Infinity);
    session.setWalkLabel(walk.name.replace(/^Xandra_/, '').toLowerCase());
    console.info('[animation] clips', JSON.stringify(clips.map(c => ({ name: c.name, duration: +c.duration.toFixed(3) }))), 'walk', walk.name, 'rootMotionRemoved', +removed.toFixed(2));
  } else {
    errEl.textContent = 'No animation clips found in ' + MODEL_URL;
  }
  perf.instrument(root); perf.event('model-load-complete', { elapsedMs: performance.now() - loadStart });
}, undefined, error => { perf.event('model-load-error'); errEl.textContent = 'Model load failed: ' + (error?.message || error); });

/** Per frame: stick -> walk weight/timeScale. Walk only while the right stick is pushed up. */
function updateWalk(dt) {
  session.setGamepad(readGamepad());
  lastSample = session.sample();
  sticksUi.paint(lastSample);
  if (!mixer || !walkAction) return;
  const walk = lastSample.walk;
  if (walk.active) {
    if (!walkAction.isRunning() || walkWeight === 0) walkAction.reset().play();
    walkAction.setEffectiveTimeScale(walk.timeScale); // speed follows the stick every frame
    walkWeight = Math.min(1, walkWeight + dt / FADE_IN_S);
  } else if (walkWeight > 0) {
    // Keep the last speed while fading back to the rest pose.
    walkWeight = Math.max(0, walkWeight - dt / FADE_OUT_S);
    if (walkWeight === 0) walkAction.stop();   // rest pose restored by the mixer
  }
  walkAction.setEffectiveWeight(walkWeight);
  mixer.update(dt);
}

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
function animate(t) {
  requestAnimationFrame(animate); perf.frame(t); const start = performance.now();
  updateWalk(Math.min(clockDelta.getDelta(), 0.1));
  const r = lastSample.sticks.right.vertical, walk = lastSample.walk;
  statsEl.textContent = !mixer ? 'loading GLB…'
    : `ANIMATION (no physics) | right stick +X ${r.toFixed(2)} | ${walk.active ? 'WALK' : 'STOP'} | timeScale ${walkAction ? walkAction.timeScale.toFixed(2) : '-'} | weight ${walkWeight.toFixed(2)}`;
  controls.update(); renderScene(); perf.sample('frameWorkCpuMs', performance.now() - start);
}
requestAnimationFrame(animate);

// Read-only hook for automated checks (headless tests); not used by the UI.
window.__animation = {
  get ready() { return !!walkAction; },
  get clips() { return clips.map(c => ({ name: c.name, duration: c.duration })); },
  state() { return { stick: lastSample.sticks.right.vertical, active: lastSample.walk.active, timeScale: walkAction?.timeScale ?? null, weight: walkWeight, running: !!walkAction?.isRunning(), time: walkAction?.time ?? null }; },
};
