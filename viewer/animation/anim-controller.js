/**
 * Animation state machine over a three.js AnimationMixer (EXP002.1-B).
 *
 * - Clips are looked up BY NAME (the glTF stores them alphabetically).
 * - Loop flags come from the manifest (loop:true -> LoopRepeat,
 *   loop:false -> LoopOnce + clampWhenFinished = hold the final pose). A clip
 *   missing from the manifest is treated as one-shot (never assumed loopable).
 * - Blending: the target clip fades in over the state's fade time while the
 *   other visible clips are scaled down proportionally (weights always sum
 *   to 1, so the bind pose never leaks in). An action keeps its phase while its
 *   weight is > 0 (no restart when input changes); Walk_Fwd <-> Run_Fwd are
 *   phase-matched by left-foot contact when one fades in over the other.
 * - States are a table (STATES) so new ones (e.g. Squat, Jump with keys) can
 *   be added without touching the blending code.
 * - Modes: 'auto' (locomotion drives the state) or 'manual' (debugger preview
 *   of any clip, including one-shots).
 */
import * as THREE from 'three';

export const LOCOMOTION_CLIPS = ['Idle', 'Walk_Fwd', 'Run_Fwd', 'Walk_Back'];
export const EXPECTED_CLIPS = ['Idle', 'Walk_Fwd', 'Run_Fwd', 'Walk_Back', 'Squat', 'Jump', 'ToPlank', 'ToBridge'];

/** Extensible state table: fade-in time when the state is entered (s). */
export const STATES = {
  Idle: { fade: 0.30 },
  Walk: { fade: 0.25 },
  Run: { fade: 0.25 },
  Preview: { fade: 0.30 },
};

const FWD_GROUP = new Set(['Walk_Fwd', 'Run_Fwd']);

export function createAnimationController({ root, clips, manifest }) {
  const mixer = new THREE.AnimationMixer(root);
  const meta = (manifest && manifest.clips) || {};
  const entries = new Map();
  const warnings = [];
  for (const clip of clips) {
    const m = meta[clip.name];
    if (!m) warnings.push(`clip ${clip.name} not in manifest: treated as one-shot`);
    const loop = m ? m.loop === true : false;
    const action = mixer.clipAction(clip);
    action.setLoop(loop ? THREE.LoopRepeat : THREE.LoopOnce, loop ? Infinity : 1);
    action.clampWhenFinished = !loop;
    entries.set(clip.name, {
      name: clip.name, clip, action, loop, duration: clip.duration,
      natural: m && Number.isFinite(m.natural_ground_speed_mps) ? m.natural_ground_speed_mps : null,
      weight: 0, target: 0, fade: 0.25, finished: false, active: false, footPhase: null,
    });
  }
  for (const name of EXPECTED_CLIPS) if (!entries.has(name)) warnings.push(`expected clip ${name} missing from the GLB`);

  let mode = 'auto';
  let state = 'Idle';
  let previewClip = null;
  let paused = false;
  let speed = 1; // playback multiplier (debugger slider)
  const listeners = new Set();

  mixer.addEventListener('finished', (event) => {
    for (const e of entries.values()) if (e.action === event.action) { e.finished = true; listeners.forEach((fn) => fn(e.name)); }
  });

  function setTargets(targets, fade) {
    for (const e of entries.values()) {
      const t = targets[e.name] || 0;
      if (t !== e.target) { e.target = t; e.fade = Math.max(1e-3, fade); }
    }
  }

  function phaseOf(e) { return e.duration > 0 ? (((e.action.time % e.duration) + e.duration) % e.duration) / e.duration : 0; }

  function activate(e) {
    if (e.active) return; // already playing (possibly fading): keep its phase, never restart
    const a = e.action;
    a.reset();
    a.enabled = true;
    if (e.loop && FWD_GROUP.has(e.name)) {
      // Phase-match to the other forward gait if it is visible.
      const other = [...FWD_GROUP].map((n) => entries.get(n)).find((o) => o && o !== e && o.weight > 0);
      if (other) {
        let p = phaseOf(other);
        if (other.footPhase != null && e.footPhase != null) p = p - other.footPhase + e.footPhase;
        a.time = (((p % 1) + 1) % 1) * e.duration;
      }
    }
    e.finished = false;
    e.active = true;
    a.play();
  }

  /** Auto mode: called every frame with the locomotion selection. */
  function setLocomotion(next) {
    if (mode !== 'auto') return;
    const changed = next.state !== state;
    state = next.state;
    const targets = {};
    if (next.clip) targets[next.clip] = 1;
    const fade = (STATES[state] || STATES.Walk).fade;
    setTargets(targets, fade);
    if (next.clip) {
      const e = entries.get(next.clip);
      activate(e);
      e.action.setEffectiveTimeScale(next.timeScale);
    }
    for (const e of entries.values()) {
      // Clips fading out keep the speed-matched rate of their own gait.
      if (e.name !== next.clip && e.weight > 0 && e.natural && next.speedAbs != null) e.action.setEffectiveTimeScale(next.speedAbs / e.natural);
      if (e.name === 'Idle' && e.name !== next.clip) e.action.setEffectiveTimeScale(1);
    }
    return changed;
  }

  function preview(name, { restart = true } = {}) {
    const e = entries.get(name);
    if (!e) return false;
    mode = 'manual'; state = 'Preview'; previewClip = name;
    if (restart || !e.active) { e.weight = 0; e.action.stop(); e.active = false; }
    activate(e);
    e.action.setEffectiveTimeScale(1);
    setTargets({ [name]: 1 }, STATES.Preview.fade);
    // The old pose crossfades out; if nothing was visible, show the preview immediately.
    if ([...entries.values()].every((o) => o === e || o.weight === 0)) e.weight = 1;
    return true;
  }

  function setMode(next) {
    mode = next === 'manual' ? 'manual' : 'auto';
    if (mode === 'auto') { previewClip = null; state = 'Idle'; }
    else if (!previewClip) {
      // Entering manual without a selection: hold whatever is visible, frozen at speed 0 for locomotion.
      state = 'Preview';
    }
  }

  function update(dt) {
    if (paused) return;
    const d = dt * speed;
    // Crossfade: the target clip gains weight at 1/fade per second and every
    // other visible clip is scaled down proportionally, so the weights always
    // sum to 1 (no bind-pose leak) and no clip jumps by more than d / fade.
    const target = [...entries.values()].find((e) => e.target === 1);
    if (target) {
      const before = target.weight;
      const after = Math.min(1, before + d / target.fade);
      const rest = 1 - before;
      let others = 0;
      for (const e of entries.values()) if (e !== target) others += e.weight;
      const scale = others > 0 ? (1 - after) / others : 0;
      target.weight = others > 0 || rest === 0 ? after : 1;
      for (const e of entries.values()) if (e !== target) { e.weight *= scale; if (e.weight < 1e-4) e.weight = 0; }
      if (target.weight > 1 - 1e-4) { target.weight = 1; for (const e of entries.values()) if (e !== target) e.weight = 0; }
    } else {
      for (const e of entries.values()) if (e.weight > 0) e.weight = Math.max(0, e.weight - d / e.fade);
    }
    for (const e of entries.values()) {
      if (e.weight === 0 && e.target === 0) {
        if (e.active) { e.action.stop(); e.active = false; e.finished = false; }
      } else {
        e.action.enabled = true;
        e.action.setEffectiveWeight(e.weight);
      }
    }
    mixer.update(d);
  }

  /**
   * Find, per locomotion clip, the phase where the left foot is furthest
   * forward (character-local +Z). Used to phase-match Walk_Fwd <-> Run_Fwd.
   * Runs once at load, before anything is shown.
   */
  function analyzeFootPhases(footBone, samples = 48) {
    if (!footBone) return {};
    const out = {};
    const tmp = new THREE.Vector3();
    root.updateMatrixWorld(true);
    const inv = new THREE.Matrix4().copy(root.matrixWorld).invert();
    for (const name of ['Walk_Fwd', 'Run_Fwd', 'Walk_Back']) {
      const e = entries.get(name);
      if (!e) continue;
      mixer.stopAllAction();
      e.action.reset(); e.action.setEffectiveWeight(1); e.action.setEffectiveTimeScale(1); e.action.play();
      let best = -Infinity, bestPhase = 0;
      for (let i = 0; i < samples; i++) {
        e.action.time = (i / samples) * e.duration;
        mixer.update(0);
        root.updateMatrixWorld(true);
        footBone.getWorldPosition(tmp).applyMatrix4(inv);
        const z = name === 'Walk_Back' ? -tmp.z : tmp.z;
        if (z > best) { best = z; bestPhase = i / samples; }
      }
      e.footPhase = bestPhase; out[name] = bestPhase;
    }
    mixer.stopAllAction();
    for (const e of entries.values()) { e.weight = 0; e.target = 0; e.active = false; e.finished = false; }
    return out;
  }

  function snapshot() {
    const clipsOut = [];
    let sum = 0;
    for (const e of entries.values()) sum += e.weight;
    for (const e of entries.values()) {
      const w = sum > 0 ? e.weight / sum : 0;
      clipsOut.push({
        name: e.name, weight: w, target: e.target, timeScale: e.action.timeScale,
        time: e.action.time, duration: e.duration, phase: phaseOf(e), loop: e.loop,
        running: e.action.isRunning(), active: e.active, finished: e.finished, natural: e.natural,
      });
    }
    return { mode, state, previewClip, paused, speed, clips: clipsOut, warnings: [...warnings] };
  }

  return {
    mixer,
    entries,
    setLocomotion,
    preview,
    setMode,
    update,
    analyzeFootPhases,
    snapshot,
    onFinished(fn) { listeners.add(fn); return () => listeners.delete(fn); },
    get mode() { return mode; },
    get state() { return state; },
    get paused() { return paused; },
    set paused(v) { paused = !!v; },
    get speed() { return speed; },
    set speed(v) { speed = Math.max(0, Math.min(4, Number(v) || 0)); },
    has(name) { return entries.has(name); },
    get available() { return new Set(entries.keys()); },
    get warnings() { return [...warnings]; },
  };
}
