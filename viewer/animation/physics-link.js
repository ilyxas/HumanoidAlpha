// EXPERIMENTAL (branch exp/assisted-stand-claude): optional PHYSICS toggle for Animation mode.
// When on, Xandra's skin follows the MuJoCo pose streamed by physics/control_console.py
// (--anim-physics), and the keys start clips on the server: the clip becomes the target of the
// stand-v2 controller and MuJoCo computes what actually happens (she can fall). When off, the
// page is the usual physics-free Animation mode.
import * as THREE from 'three';

const QC = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), -Math.PI / 2);
const zupQuatToYup = wxyz => QC.clone().multiply(new THREE.Quaternion(wxyz[1], wxyz[2], wxyz[3], wxyz[0]));
const yupPosFromZup = xyz => new THREE.Vector3(xyz[0], xyz[2], -xyz[1]);

// Physical key code -> clip. Held movement keys loop a locomotion clip; release returns to stand.
const ONE_SHOTS = { Space: 'Jump', Digit1: 'ToPlank', Digit2: 'ToBridge', Digit3: 'Squat', Numpad1: 'ToPlank', Numpad2: 'ToBridge', Numpad3: 'Squat' };

export function createPhysicsLink({ wsPort, onStatus }) {
  let socket = null, reconnect = null, root = null, bones = new Map();
  let enabled = false, connected = false, lastState = null, heldLoop = null;
  const held = { KeyW: false, KeyS: false, KeyI: false };
  let shift = false;
  const _pw = new THREE.Quaternion(), _inv = new THREE.Quaternion(), _cur = new THREE.Vector3();

  function setModel(gltfRoot) {
    root = gltfRoot; bones = new Map();
    root.traverse(o => { if (o.isBone) bones.set(o.name, o); });
  }

  function send(msg) { if (socket && socket.readyState === 1) socket.send(JSON.stringify(msg)); }

  function applyPose(pose) {
    if (!root || !pose?.bone_world_quat) return;
    for (const name of pose.apply_order || Object.keys(pose.bone_world_quat)) {
      const wxyz = pose.bone_world_quat[name], bone = bones.get(name);
      if (!wxyz || !bone) continue;
      const desired = zupQuatToYup(wxyz);
      if (bone.parent) { bone.parent.getWorldQuaternion(_pw).normalize(); _inv.copy(_pw).invert(); bone.quaternion.copy(_inv).multiply(desired).normalize(); }
      else bone.quaternion.copy(desired);
      bone.updateMatrix();
    }
    const pelvis = bones.get('pelvis'), target = pose.bone_world_pos?.pelvis || pose.root_pos;
    if (pelvis && target) { root.updateMatrixWorld(true); pelvis.getWorldPosition(_cur); root.position.add(yupPosFromZup(target).sub(_cur)); }
  }

  function status() {
    const s = lastState?.stand;
    let text = !enabled ? 'PHYSICS OFF' : !connected ? 'PHYSICS · connecting…' : 'PHYSICS ON';
    if (enabled && connected && s) {
      const feet = (s.stance_hint || []).map((h, i) => (h ? 'LR'[i] : '-')).join('');
      text += ` · ${s.clip ? `${s.clip} ${s.clip_t?.toFixed(1)}s` : 'stand'} · feet ${feet} · torque scale ${Number(s.task_scale).toFixed(2)}`
        + (s.fallen ? ' · FALLEN (R = stand up)' : '');
    }
    onStatus?.(text, enabled);
  }

  function connect() {
    if (socket) return;
    socket = new WebSocket(`ws://${location.hostname || '127.0.0.1'}:${wsPort}`);
    socket.onopen = () => { connected = true; send({ op: 'assist', value: 'reset' }); status(); };
    socket.onmessage = event => {
      let m; try { m = JSON.parse(event.data); } catch { return; }
      if (m.type === 'pose') { lastState = m.state; if (enabled) applyPose(m.pose); status(); }
      else if (m.type === 'error') console.warn('[physics]', m.message);
    };
    socket.onclose = () => { connected = false; socket = null; status(); if (enabled) reconnect = setTimeout(connect, 1500); };
    socket.onerror = () => {};
  }

  function setEnabled(on) {
    enabled = on;
    if (on) { connect(); if (connected) send({ op: 'assist', value: 'reset' }); }
    else { send({ op: 'clip_stop' }); heldLoop = null; if (root) root.position.set(0, 0, 0); }
    status();
  }

  function updateLoop() {
    const want = held.KeyW ? (shift ? 'Run_Fwd' : 'Walk_Fwd') : held.KeyS ? 'Walk_Back' : held.KeyI ? 'Idle' : null;
    if (want === heldLoop) return;
    heldLoop = want;
    send(want ? { op: 'clip', name: want, loop: true } : { op: 'clip_stop' });
  }

  /** Returns true when the key was consumed by physics mode. */
  function onKey(e) {
    if (!enabled) return false;
    const down = e.type === 'keydown';
    if (e.key === 'Shift') { shift = down; updateLoop(); return true; }
    if (ONE_SHOTS[e.code]) { e.preventDefault(); if (down && !e.repeat) { heldLoop = null; send({ op: 'clip', name: ONE_SHOTS[e.code], loop: false }); } return true; }
    if (e.code === 'KeyR') { if (down && !e.repeat) { heldLoop = null; send({ op: 'assist', value: 'reset' }); } return true; }
    if (e.code in held) { held[e.code] = down; shift = e.shiftKey; if (down) e.preventDefault(); updateLoop(); return true; }
    return ['KeyA', 'KeyD', 'CapsLock'].includes(e.code);  // no turning/strafing clips; swallow
  }

  function pelvisWorld(out) { const p = bones.get('pelvis'); if (!p) return null; p.getWorldPosition(out); return out; }

  return { setModel, setEnabled, onKey, pelvisWorld, get enabled() { return enabled; }, get state() { return lastState; } };
}
