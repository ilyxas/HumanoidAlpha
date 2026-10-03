import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { createProfiler } from './observation-perf.js';
import { createHumanControls } from './human/attach.js';

const params = new URLSearchParams(location.search);
//const perf = createProfiler(params.get('profile') === '1');
const perf = createProfiler(true)
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
// Fixed world-space camera poses. Never derived from body state.
const target = new THREE.Vector3(0, .7, 0);
const presets = { FRONT: [0, 1.6, 5.5], LEFT: [5.5, 1.6, 0], BACK: [0, 1.6, -5.5], RIGHT: [-5.5, 1.6, 0], TOP: [0, 6.2, 0], RESET: [3.6, 2.5, 4.6] };
const statsEl = document.getElementById('stats');
let skinnedMeshes = 0;
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
// No joint picking or lab markup. Human stick commands live in viewer/human/.
let root = null;
const bonesByName = new Map();
const getBone = name => bonesByName.get(name) || null;
const QC = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), -Math.PI / 2);
function zupQuatToYup(wxyz) { return QC.clone().multiply(new THREE.Quaternion(wxyz[1], wxyz[2], wxyz[3], wxyz[0])); }
function yupPosFromZup(xyz) { return new THREE.Vector3(xyz[0], xyz[2], -xyz[1]); }
const _parentWorld = new THREE.Quaternion(), _desired = new THREE.Quaternion(), _invParent = new THREE.Quaternion();
function applyPhysicsPose(pose) {
  if (!pose || !pose.bone_world_quat) return;
  const order = pose.apply_order || Object.keys(pose.bone_world_quat);

  // Root translate: move model root so pelvis matches (Y-up)
  if (pose.apply_root_translate && pose.root_pos && root) {
    const target = yupPosFromZup(pose.bone_world_pos?.pelvis || pose.root_pos);
    const pelvis = getBone('pelvis');
    if (pelvis) {
      // After bone updates we correct; approximate: set root offset
      // Store bind pelvis world at first pose
      root.updateMatrixWorld(true);
      const cur = new THREE.Vector3();
      pelvis.getWorldPosition(cur);
      const delta = target.clone().sub(cur);
      root.position.add(delta);
    }
  }

  for (const name of order) {
    const wxyz = pose.bone_world_quat[name];
    if (!wxyz) continue;
    const bone = getBone(name);
    if (!bone) continue;
    _desired.copy(zupQuatToYup(wxyz));
    if (bone.parent) {
      bone.parent.getWorldQuaternion(_parentWorld).normalize();
      _invParent.copy(_parentWorld).invert();
      bone.quaternion.copy(_invParent).multiply(_desired).normalize();
    } else {
      bone.quaternion.copy(_desired);
    }
    bone.updateMatrix();
  }
  // Second pass root translate after orientations
  if (pose.apply_root_translate && (pose.bone_world_pos?.pelvis || pose.root_pos) && root) {
    root.updateMatrixWorld(true);
    const pelvis = getBone('pelvis');
    if (pelvis) {
      const target = yupPosFromZup(pose.bone_world_pos?.pelvis || pose.root_pos);
      const cur = new THREE.Vector3();
      pelvis.getWorldPosition(cur);
      root.position.add(target.sub(cur));
    }
  }

}

let socket, reconnect;
const human = createHumanControls(document);
function connect() {
  // Optional local test port. Pose stream is unchanged. human_cmd is a separate op.
  const port = params.get('wsPort') || '8766';
  if (!/^\d{1,5}$/.test(port) || Number(port) > 65535) return;
  socket = new WebSocket(`ws://${location.hostname || '127.0.0.1'}:${port}`);
  human.bindSocket(socket);
  socket.onopen = () => perf.event('connected');
  socket.onmessage = event => {
    const t = performance.now();
    try {
      const message = JSON.parse(event.data);
      if (message.type === 'inventory') { human.onInventory(message.inventory); return; }
      if (message.type !== 'pose') return;
      perf.pose();
      const begin = performance.now(); applyPhysicsPose(message.pose); perf.sample('poseApplyCpuMs', performance.now() - begin);
      perf.sample('poseHandlerCpuMs', performance.now() - t);
    } catch { perf.event('message-error'); }
  };
  socket.onclose = () => { perf.event('disconnected'); reconnect = setTimeout(connect, 1500); };
  socket.onerror = () => perf.event('connection-error');
}
window.addEventListener('pagehide', () => { clearTimeout(reconnect); human.stop(); if (socket) { socket.onclose = null; socket.close(); } });
const loadStart = performance.now(); perf.event('model-load-start');
new GLTFLoader().load('../assets/Xandra.glb', gltf => {
  root = gltf.scene;
  root.traverse(o => { if (o.isBone) bonesByName.set(o.name, o); if (o.isMesh) { o.castShadow = true; o.receiveShadow = true; } });
  perf.instrument(root); scene.add(root); perf.event('model-load-complete', { elapsedMs: performance.now() - loadStart }); connect();
  statsEl.textContent = `GLB OK | bones=${bonesByName.size} skinnedMeshes=${skinnedMeshes}`;
}, undefined, () => perf.event('model-load-error'));

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
  // Copy the currently displayed buffers, without advancing scene or camera.
  const shot = document.createElement('canvas'); shot.width = canvas.width; shot.height = canvas.height;
  const ctx = shot.getContext('2d'); ctx.drawImage(canvas, 0, 0); ctx.drawImage(overlay, 0, 0, shot.width, shot.height);
  const now = new Date(), pad = n => String(n).padStart(2, '0');
  const stamp = `${now.getFullYear()}-${pad(now.getMonth() + 1)}-${pad(now.getDate())}_${pad(now.getHours())}-${pad(now.getMinutes())}-${pad(now.getSeconds())}`;
  const link = document.createElement('a'); link.download = `observation_${stamp}.png`; link.href = shot.toDataURL('image/png'); document.body.append(link); link.click(); link.remove();
};
function renderScene() { const start = performance.now(); renderer.render(scene, camera); perf.sample('rendererCpuMs', performance.now() - start); }
function animate(t) { 
  requestAnimationFrame(animate); perf.frame(t); const start = performance.now(); 
  statsEl.textContent = `performance=${performance.now()-start} `;
controls.update(); renderScene(); perf.sample('frameWorkCpuMs', performance.now() - start); }
requestAnimationFrame(animate);
