/**
 * Humanoid Alpha v0.2 Three.js viewer
 *
 * Coordinate notes (DOCUMENTED):
 * - MuJoCo + Blender calibration: Z-up. mapping_full gives bone_world_quat in Z-up.
 * - GLB exported with export_yup=True → Three.js scene is Y-up.
 * - Convert orientation: q_yup = qC * q_zup (absolute bone frame; verified against GLB rest)
 *   where C maps (x,y,z)_z → (x,z,-y)_y  ≡ Rx(-90°)
 * - Apply: bone.quaternion = inv(parentWorldQuat) * q_yup_desired
 *   (world-orientation drive; same idea as apply_full_blender set_bone_world_quat)
 * - Manual knee FACT (Blender local): calf_r Quaternion((0,0,1), -q) i.e. local -Z.
 *   After Y-up export, local bone axes are whatever the exporter wrote; we store
 *   rest local quats and apply delta in bone-local space empirically (axis toggle).
 */
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { SkeletonHelper } from 'three';

const canvas = document.getElementById('c');
const statsEl = document.getElementById('stats');
const errEl = document.getElementById('err');

const renderer = new THREE.WebGLRenderer({ canvas, antialias: true, preserveDrawingBuffer: true });
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
renderer.outputColorSpace = THREE.SRGBColorSpace;
renderer.toneMapping = THREE.ACESFilmicToneMapping;

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x1a1a22);

const camera = new THREE.PerspectiveCamera(40, 2, 0.05, 50);
camera.position.set(1.6, 1.2, 2.4);

const controls = new OrbitControls(camera, canvas);
controls.target.set(0, 0.95, 0);
controls.update();

scene.add(new THREE.HemisphereLight(0xffffff, 0x334455, 1.2));
const dir = new THREE.DirectionalLight(0xffffff, 1.4);
dir.position.set(2, 4, 3);
scene.add(dir);
scene.add(new THREE.GridHelper(4, 20, 0x444466, 0x2a2a3a));

// Z-up → Y-up world-basis conversion (Rx -90°)
const QC = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), -Math.PI / 2);


function zupQuatToYup(wxyz) {
  // wxyz array from MuJoCo/Blender
  const q = new THREE.Quaternion(wxyz[1], wxyz[2], wxyz[3], wxyz[0]); // THREE is xyzw
  // Absolute bone frame changes world basis only. GLB retains bone-local axes.
  // Verified against all 23 GLB rest orientations; conjugation introduces 90° error.
  return QC.clone().multiply(q);
}

function yupPosFromZup(xyz) {
  // (x,y,z)_z → (x,z,-y)_y
  return new THREE.Vector3(xyz[0], xyz[2], -xyz[1]);
}

let root = null;
let skeletonHelper = null;
let bonesByName = new Map();
let restLocalQuat = new Map();
let restRootPos = null;
let mode = 'manual'; // manual | physics
let ws = null;
let lastPoseTime = 0;
let fps = 0, frames = 0, lastFpsT = performance.now();
let physicsMsgHz = 0, physicsMsgCount = 0, lastPhysT = performance.now();
let latencyMs = null;
let consoleErrors = [];
let skinnedMeshes = 0;
let mjDebugLines = null;

window.addEventListener('error', (e) => {
  consoleErrors.push(String(e.message || e));
  errEl.textContent = consoleErrors.slice(-5).join('\n');
});

function resize() {
  const w = canvas.clientWidth || window.innerWidth;
  const h = canvas.clientHeight || window.innerHeight;
  renderer.setSize(w, h, false);
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
}
window.addEventListener('resize', resize);
resize();

function indexBones(obj) {
  bonesByName.clear();
  restLocalQuat.clear();
  obj.traverse((o) => {
    if (o.isBone) {
      bonesByName.set(o.name, o);
      restLocalQuat.set(o.name, o.quaternion.clone());
    }
    if (o.isSkinnedMesh) skinnedMeshes++;
  });
}

function getBone(name) {
  return bonesByName.get(name) || null;
}

function resetToRest() {
  for (const [name, q] of restLocalQuat) {
    const b = bonesByName.get(name);
    if (b) b.quaternion.copy(q);
  }
  if (root && restRootPos) root.position.copy(restRootPos);
  document.getElementById('knee').value = 0;
  document.getElementById('shoulder').value = 0;
  document.getElementById('shoulderRoll').value = 0;
  document.getElementById('vKnee').textContent = '0.00';
  document.getElementById('vSh').textContent = '0.00';
  document.getElementById('vShR').textContent = '0.00';
}

/**
 * Manual drive: apply delta on rest local quat.
 * Knee FACT Blender: axis local Z, angle -q (flexion positive q → rotate -Z).
 * Empirically after Y-up export we try local axes; default Z then fallback documented in VIEWER_MANUAL.
 */
function applyManual() {
  if (mode !== 'manual') return;
  const knee = parseFloat(document.getElementById('knee').value);
  const sh = parseFloat(document.getElementById('shoulder').value);
  const shR = parseFloat(document.getElementById('shoulderRoll').value);
  document.getElementById('vKnee').textContent = knee.toFixed(2);
  document.getElementById('vSh').textContent = sh.toFixed(2);
  document.getElementById('vShR').textContent = shR.toFixed(2);

  const calf = getBone('calf_r');
  if (calf) {
    const rest = restLocalQuat.get('calf_r');
    // FACT: Quaternion(axis=(0,0,1), angle=-q) in Blender bone local
    const dq = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 0, 1), -knee);
    calf.quaternion.copy(rest).multiply(dq);
  }

  const ua = getBone('upperarm_r');
  if (ua) {
    const rest = restLocalQuat.get('upperarm_r');
    // Shoulder manual: local Y pitch (raise arm) + local Z roll.
    // Axes chosen empirically on Y-up glTF export (see VIEWER_MANUAL.md).
    const dq = new THREE.Quaternion()
      .setFromAxisAngle(new THREE.Vector3(0, 1, 0), sh)
      .multiply(new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 0, 1), shR));
    ua.quaternion.copy(rest).multiply(dq);
  }
}

/**
 * Apply calibrated world quats from mapping_full (Z-up) onto Three.js bones (Y-up).
 * Parents before children via apply_order.
 */
const _parentWorld = new THREE.Quaternion();
const _desired = new THREE.Quaternion();
const _invParent = new THREE.Quaternion();

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

  // Optional MuJoCo debug capsule/skeleton lines
  if (document.getElementById('mjDebug').checked && pose.bone_world_pos) {
    updateMjDebug(pose);
  }
}

function updateMjDebug(pose) {
  if (mjDebugLines) { scene.remove(mjDebugLines); mjDebugLines.geometry.dispose(); mjDebugLines.material.dispose(); }
  const pts = [];
  const pairs = [
    ['pelvis', 'spine_01'], ['spine_01', 'spine_05'], ['spine_05', 'head'],
    ['pelvis', 'thigh_r'], ['thigh_r', 'calf_r'], ['calf_r', 'foot_r'],
    ['pelvis', 'thigh_l'], ['thigh_l', 'calf_l'], ['calf_l', 'foot_l'],
    ['spine_05', 'upperarm_r'], ['upperarm_r', 'lowerarm_r'], ['lowerarm_r', 'hand_r'],
    ['spine_05', 'upperarm_l'], ['upperarm_l', 'lowerarm_l'], ['lowerarm_l', 'hand_l'],
  ];
  const pos = pose.bone_world_pos || {};
  for (const [a, b] of pairs) {
    if (!pos[a] || !pos[b]) continue;
    const pa = yupPosFromZup(pos[a]);
    const pb = yupPosFromZup(pos[b]);
    pts.push(pa, pb);
  }
  const geo = new THREE.BufferGeometry().setFromPoints(pts);
  mjDebugLines = new THREE.LineSegments(geo, new THREE.LineBasicMaterial({ color: 0x00ff88 }));
  scene.add(mjDebugLines);
}

function connectWS() {
  if (ws) try { ws.close(); } catch (_) {}
  const port = new URLSearchParams(location.search).get('wsPort') || '8766';
  const url = `ws://${location.hostname || '127.0.0.1'}:${port}`;
  statsEl.textContent += `\nWS → ${url}`;
  ws = new WebSocket(url);
  ws.onopen = () => { errEl.textContent = ''; window.dispatchEvent(new CustomEvent('control-connection', {detail: true})); };
  ws.onerror = () => { errEl.textContent = 'WebSocket error (is bridge running?)'; };
  ws.onclose = () => { errEl.textContent = 'Disconnected — controls disabled'; window.dispatchEvent(new CustomEvent('control-connection', {detail: false})); };
  ws.onmessage = (ev) => {
    physicsMsgCount++;
    try {
      const msg = JSON.parse(ev.data);
      if(msg.type==='inventory')controlInventory=msg.inventory;
      window.dispatchEvent(new CustomEvent('control-message', {detail: msg}));
      if (msg.type === 'pose') {
        if (msg.t_send != null) latencyMs = performance.now() - msg.t_send_perf;
        // server sends t_wall_ms; estimate one-way if present
        if (msg.t_wall_ms != null) {
          // can't sync clocks perfectly; use client receive gap
        }
        lastPoseTime = msg.sim_time;
        applyPhysicsPose(msg.pose);
        latestPhysicalState = msg.state;
        latestPose = msg.pose;
        updateSelection();
        if (msg.t_send_ms != null) {
          latencyMs = Date.now() - msg.t_send_ms; // rough if same machine
        }
      }
    } catch (e) {
      consoleErrors.push(String(e));
    }
  };
}

const loader = new GLTFLoader();
loader.load('../assets/Xandra.glb', (gltf) => {
  root = gltf.scene;
  scene.add(root);
  restRootPos = root.position.clone();
  indexBones(root);
  skeletonHelper = new SkeletonHelper(root);
  skeletonHelper.visible = document.getElementById('skel').checked;
  scene.add(skeletonHelper);
  resetToRest();
  frameCharacter();
  document.getElementById('btnPhysics').click();
  statsEl.textContent = `GLB OK | bones=${bonesByName.size} skinnedMeshes=${skinnedMeshes}`;
}, undefined, (e) => {
  errEl.textContent = 'GLB load failed: ' + e;
});

document.getElementById('knee').oninput = applyManual;
document.getElementById('shoulder').oninput = applyManual;
document.getElementById('shoulderRoll').oninput = applyManual;
document.getElementById('btnReset').onclick = () => { resetToRest(); applyManual(); };
document.getElementById('skel').onchange = (e) => { if (skeletonHelper) skeletonHelper.visible = e.target.checked; };
document.getElementById('btnManual').onclick = () => {
  mode = 'manual';
  document.getElementById('btnManual').classList.add('active');
  document.getElementById('btnPhysics').classList.remove('active');
  if (ws) try { ws.close(); } catch (_) {}
  resetToRest();
};
document.getElementById('btnPhysics').onclick = () => {
  mode = 'physics';
  document.getElementById('btnPhysics').classList.add('active');
  document.getElementById('btnManual').classList.remove('active');
  connectWS();
};
document.getElementById('btnShot').onclick = () => {
  const a = document.createElement('a');
  a.download = `shot_${Date.now()}.png`;
  a.href = canvas.toDataURL('image/png');
  a.click();
};

let latestPhysicalState = null, latestPose = null, selectedJoint = null, controlInventory = null;
const selectionGroup = new THREE.Group();
scene.add(selectionGroup);
function clearSelection() {
  while (selectionGroup.children.length) {
    const o = selectionGroup.children[0]; selectionGroup.remove(o);
    o.geometry?.dispose(); o.material?.dispose();
  }
}
function marker(position, color, radius) {
  const o = new THREE.Mesh(new THREE.SphereGeometry(radius, 10, 8), new THREE.MeshBasicMaterial({color, depthTest:false}));
  o.position.copy(position); o.renderOrder = 10; selectionGroup.add(o);
}
function updateSelection() {
  clearSelection();
  if (!selectedJoint || !latestPhysicalState || !root) return;
  root.updateMatrixWorld(true);
  const anchor = latestPhysicalState.joint_anchors[selectedJoint.id];
  if (anchor) marker(yupPosFromZup(anchor), 0xffbe55, 0.018);
  for (const name of selectedJoint.visual_bones) {
    const bone = getBone(name); if (!bone) continue;
    const point = bone.getWorldPosition(new THREE.Vector3());
    marker(point, 0x60efff, 0.012);
    const child = bone.children.find(o => o.isBone);
    if (child) {
      const geo = new THREE.BufferGeometry().setFromPoints([point,child.getWorldPosition(new THREE.Vector3())]);
      const line = new THREE.Line(geo,new THREE.LineBasicMaterial({color:0x60efff,depthTest:false}));
      line.renderOrder=11; selectionGroup.add(line);
    }
  }
}
function frameCharacter() {
  if (!root) return;
  root.updateMatrixWorld(true);
  const box = new THREE.Box3();
  for (const bone of bonesByName.values()) box.expandByPoint(bone.getWorldPosition(new THREE.Vector3()));
  const center = box.getCenter(new THREE.Vector3());
  const size = box.getSize(new THREE.Vector3()).length();
  controls.target.copy(center);
  camera.position.copy(center).add(new THREE.Vector3(0, size*0.12, Math.max(1.5,size*1.7)));
  controls.update();
}
function diagnostics() {
  if (!selectedJoint || !root || !latestPose) return [];
  root.updateMatrixWorld(true);
  return selectedJoint.visual_bones.map(name => {
    const bone=getBone(name), target=latestPose.bone_world_quat[name];
    if (!bone || !target) return {bone:name, present:!!bone, mapped:!!target};
    const actual=bone.getWorldQuaternion(new THREE.Quaternion()).normalize();
    const desired=zupQuatToYup(target);
    return {bone:name,present:true,mapped:true,world_error_rad:actual.angleTo(desired),
      visual_position:bone.getWorldPosition(new THREE.Vector3()).toArray(),
      physical_position:latestPose.bone_world_pos[name] || null};
  });
}
// Expose for headless capture
window.__HA = {
  send: cmd => { if (ws?.readyState !== WebSocket.OPEN) throw new Error('Not connected'); ws.send(JSON.stringify(cmd)); },
  select: joint => { selectedJoint=joint; updateSelection(); },
  diagnostics, frameCharacter,
  latestState: () => latestPhysicalState,
  latestPose: () => latestPose,

  setKnee: (v) => { document.getElementById('knee').value = v; applyManual(); },
  setShoulder: (p, r = 0) => {
    document.getElementById('shoulder').value = p;
    document.getElementById('shoulderRoll').value = r;
    applyManual();
  },
  reset: () => resetToRest(),
  modeManual: () => document.getElementById('btnManual').click(),
  modePhysics: () => document.getElementById('btnPhysics').click(),
  getStats: () => ({
    fps, bones: bonesByName.size, skinnedMeshes, mode,
    physicsMsgHz, latencyMs, lastPoseTime, consoleErrors: consoleErrors.slice(),
  }),
  ready: () => bonesByName.size > 0,
  measureRestErrors: (pose) => {
    // Apply pose then compare bone world quat to converted target
    applyPhysicsPose(pose);
    root.updateMatrixWorld(true);
    const out = {};
    const q = new THREE.Quaternion();
    for (const [name, wxyz] of Object.entries(pose.bone_world_quat || {})) {
      const b = bonesByName.get(name);
      if (!b) continue;
      b.getWorldQuaternion(q);
      const des = zupQuatToYup(wxyz);
      // geodesic angle
      let dot = Math.abs(q.w*des.w + q.x*des.x + q.y*des.y + q.z*des.z);
      dot = Math.min(1, Math.max(0, dot));
      out[name] = 2 * Math.acos(dot);
    }
    return out;
  },
  compareRestLocals: () => {
    const out = {};
    for (const [name, rest] of restLocalQuat) {
      const b = bonesByName.get(name);
      if (!b) continue;
      const q = b.quaternion;
      let dot = Math.abs(q.w*rest.w + q.x*rest.x + q.y*rest.y + q.z*rest.z);
      dot = Math.min(1, Math.max(0, dot));
      out[name] = 2 * Math.acos(dot);
    }
    return out;
  },
};

function animate(t) {
  requestAnimationFrame(animate);
  frames++;
  if (t - lastFpsT >= 1000) {
    fps = frames;
    frames = 0;
    lastFpsT = t;
    physicsMsgHz = physicsMsgCount;
    physicsMsgCount = 0;
  }
  if (mode === 'manual') applyManual();
  if (mjDebugLines) mjDebugLines.visible = document.getElementById('mjDebug').checked;
  controls.update();
  renderer.render(scene, camera);
  if (bonesByName.size) {
    statsEl.textContent =
      `mode=${mode}  FPS=${fps}  bones=${bonesByName.size}  skinned=${skinnedMeshes}\n` +
      `physMsgHz=${physicsMsgHz}  latencyMs≈${latencyMs != null ? latencyMs.toFixed(1) : 'n/a'}  sim_t=${lastPoseTime.toFixed?.(3) ?? lastPoseTime}`;
  }
}
requestAnimationFrame(animate);

// Raycast the current deformed skin, then resolve weighted skin bones to
// existing physical controls. Unmapped fingers/twist bones walk up their rig
// ancestry. Multiple wrist/ankle hinges remain available in the opened group.
const pickRay = new THREE.Raycaster();
let pickStart=null;
function jointsForSkinBone(bone){
  while(bone){
    const direct=controlInventory.joints.filter(j=>j.visual_bones[0]===bone.name);
    if(direct.length)return direct;
    const derived=controlInventory.joints.filter(j=>j.visual_bones.includes(bone.name));
    if(derived.length){
      const side=bone.name.endsWith('_l')?'_l':bone.name.endsWith('_r')?'_r':null;
      const sameSide=side?derived.filter(j=>j.body.endsWith(side)):[];
      return sameSide.length?sameSide:derived;
    }
    bone=bone.parent;
  }
  return [];
}
function pickBody(clientX,clientY){
  if(!root||!controlInventory)return;
  const rect=canvas.getBoundingClientRect();
  pickRay.setFromCamera(new THREE.Vector2((clientX-rect.left)/rect.width*2-1,-(clientY-rect.top)/rect.height*2+1),camera);
  root.updateMatrixWorld(true);
  const meshes=[];
  root.traverse(o=>{if(o.isSkinnedMesh&&o.visible){o.skeleton.update();o.computeBoundingSphere();if(o.boundingBox)o.computeBoundingBox();meshes.push(o);}});
  const hits=pickRay.intersectObjects(meshes,false);
  for(const hit of hits){
    const indices=hit.object.geometry.getAttribute('skinIndex'),weights=hit.object.geometry.getAttribute('skinWeight');
    if(!hit.face||!indices||!weights)continue;
    const bary=hit.barycoord?.toArray()||[1/3,1/3,1/3];
    const scores=new Map();
    [hit.face.a,hit.face.b,hit.face.c].forEach((vertex,v)=>{
      for(let k=0;k<4;k++){
        const weight=weights.getComponent(vertex,k)*bary[v];if(weight<=0)continue;
        const bone=hit.object.skeleton.bones[indices.getComponent(vertex,k)];
        const joints=jointsForSkinBone(bone);
        if(!joints.length)continue;
        const key=joints.map(j=>j.id).join(',');
        const previous=scores.get(key)||{score:0,jointIds:joints.map(j=>j.id)};
        previous.score+=weight;scores.set(key,previous);
      }
    });
    const winner=[...scores.values()].sort((a,b)=>b.score-a.score)[0];
    if(winner){window.dispatchEvent(new CustomEvent('control-pick',{detail:winner}));return;}
  }
}
canvas.addEventListener('pointerdown',event=>{
  if(event.button===0&&event.isPrimary)pickStart={id:event.pointerId,x:event.clientX,y:event.clientY,moved:false};
  else pickStart=null;
});
canvas.addEventListener('pointermove',event=>{
  if(pickStart&&Math.hypot(event.clientX-pickStart.x,event.clientY-pickStart.y)>6)pickStart.moved=true;
});
canvas.addEventListener('pointerup',event=>{
  const start=pickStart;pickStart=null;
  if(start&&start.id===event.pointerId&&!start.moved&&event.button===0)pickBody(event.clientX,event.clientY);
});
canvas.addEventListener('pointercancel',()=>{pickStart=null;});
