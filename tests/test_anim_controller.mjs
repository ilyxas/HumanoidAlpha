// One-shot arbitration in the animation controller (EXP002.1-C). Run: node tests/test_anim_controller.mjs
// Uses synthetic clips with the real manifest's names, loop flags and durations.
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import * as THREE from 'three';
import { createAnimationController, STATES } from '../viewer/animation/anim-controller.js';

const manifest = JSON.parse(readFileSync(new URL('../assets/Xandra_Animated.manifest.json', import.meta.url)));
function make() {
  const root = new THREE.Group(); const bone = new THREE.Object3D(); bone.name = 'pelvis'; root.add(bone);
  const clips = Object.entries(manifest.clips).map(([name, m]) => {
    const d = m.duration_s ?? m.duration ?? (m.frames ? m.frames / (m.fps || 30) : 1);
    return new THREE.AnimationClip(name, d, [new THREE.VectorKeyframeTrack('pelvis.position', [0, d], [0, 1, 0, 0, 0.9, 0])]);
  });
  return { anim: createAnimationController({ root, clips, manifest }), clips };
}
const dt = 1 / 30;
const weights = (anim) => Object.fromEntries(anim.snapshot().clips.map((c) => [c.name, c.weight]));
const idle = { state: 'Idle', clip: 'Idle', timeScale: 1, speedAbs: 0 };
function step(anim, n = 1, loco = idle) {
  let maxJump = 0;
  for (let i = 0; i < n; i++) {
    const before = weights(anim);
    anim.setLocomotion(loco); anim.update(dt);
    const after = weights(anim);
    const sum = Object.values(after).reduce((a, b) => a + b, 0);
    assert.ok(Math.abs(sum - 1) < 1e-6, `weights sum to 1 (${sum})`);
    for (const k in after) maxJump = Math.max(maxJump, Math.abs(after[k] - before[k]));
  }
  return maxJump;
}

const { anim, clips } = make();
const dur = Object.fromEntries(clips.map((c) => [c.name, c.duration]));
step(anim, 30);
assert.equal(weights(anim).Idle, 1);

// Jump: starts, owns the body, rejects repeats and other one-shots while playing.
let r = anim.triggerOneShot('Jump', { fadeIn: 0.25, holdS: 0, fadeOut: 0.35 });
assert.ok(r.ok, JSON.stringify(r)); assert.equal(anim.state, 'Action'); assert.equal(anim.oneShotPhase, 'play');
assert.equal(anim.setLocomotion({ state: 'Walk', clip: 'Walk_Fwd', timeScale: 1, speedAbs: 1.4 }), false, 'locomotion ignored during one-shot');
let jump = step(anim, 3);
assert.ok(jump <= dt / 0.25 + 1e-6, `smooth fade-in ${jump}`);
r = anim.triggerOneShot('Jump'); assert.equal(r.ok, false); assert.match(r.reason, /busy/);
r = anim.triggerOneShot('Squat'); assert.equal(r.ok, false); assert.match(r.reason, /busy: Jump/);
const t0 = anim.snapshot().oneShot.time; assert.ok(t0 > 0.05, 'not restarted by the rejected trigger');
r = anim.triggerOneShot('Idle'); assert.equal(r.ok, false); assert.equal(r.reason, 'not a one-shot clip');
// Completes, then returns smoothly to locomotion.
let frames = 0; while (anim.oneShotPhase && frames < 400) { jump = Math.max(jump, step(anim)); frames++; }
assert.equal(anim.oneShotPhase, null, 'Jump finished');
assert.ok(Math.abs(frames * dt - dur.Jump) < 0.2, `played its full length ${frames * dt} vs ${dur.Jump}`);
assert.ok(weights(anim).Jump > 0.8, 'return is a fade, not a cut');
r = anim.triggerOneShot('Jump'); assert.equal(r.ok, false); assert.match(r.reason, /fading out/);
jump = Math.max(jump, step(anim, 15));
assert.equal(weights(anim).Jump, 0); assert.equal(weights(anim).Idle, 1); assert.equal(anim.state, 'Idle');
assert.ok(jump <= dt / 0.25 + 1e-6, `no weight jumps ${jump}`);
assert.equal(anim.snapshot().lastOneShot.ok, false);

// ToPlank holds its final pose, then auto-returns; movement (release) ends the hold early.
r = anim.triggerOneShot('ToPlank', { fadeIn: 0.25, holdS: 2, fadeOut: 0.8 }); assert.ok(r.ok);
frames = 0; while (anim.oneShotPhase === 'play' && frames < 600) { step(anim); frames++; }
assert.equal(anim.oneShotPhase, 'hold');
r = anim.triggerOneShot('ToPlank'); assert.equal(r.ok, false); assert.match(r.reason, /already holding/);
step(anim, 30); assert.equal(anim.oneShotPhase, 'hold', 'still holding after 1 s');
step(anim, 35); assert.equal(anim.oneShotPhase, null, 'auto-return after 2 s hold');
step(anim, 40); assert.equal(weights(anim).Idle, 1);
r = anim.triggerOneShot('ToPlank', { holdS: 2, fadeOut: 0.8 }); assert.ok(r.ok);
frames = 0; while (anim.oneShotPhase === 'play' && frames < 600) { step(anim); frames++; }
assert.equal(anim.releaseOneShot(), true); assert.equal(anim.oneShotPhase, null);
assert.ok(anim.actionWeight > 0.9, 'release starts a fade');
step(anim, 40); assert.equal(anim.actionWeight, 0);
// A different one-shot may start from a hold (first-come: the hold is not "playing").
const b = make().anim; step(b, 10);
assert.ok(b.triggerOneShot('ToPlank', { holdS: 5 }).ok);
frames = 0; while (b.oneShotPhase === 'play' && frames < 600) { step(b); frames++; }
assert.ok(b.triggerOneShot('Squat', { fadeIn: 0.6 }).ok, 'another one-shot ends a hold');
assert.equal(b.oneShotPhase, 'play'); assert.equal(b.snapshot().oneShot.name, 'Squat');
step(b, 30); assert.equal(weights(b).ToPlank, 0); assert.ok(weights(b).Squat > 0.999);
// Manual mode (debugger preview): keys cannot trigger, preview still plays one-shots.
b.setMode('manual'); assert.equal(b.triggerOneShot('Jump').reason, 'manual mode');
assert.ok(b.preview('Jump')); step(b, 10); assert.ok(weights(b).Jump > 0.99);
b.setMode('auto'); assert.equal(b.oneShotPhase, null);
assert.equal(STATES.Action.fade, 0.25);
console.log('test_anim_controller OK');
