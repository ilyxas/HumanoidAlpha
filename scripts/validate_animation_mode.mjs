// EXP002.1-B validation against the REAL running app. Start it first: ./START.command animation --no-open
// Then: node scripts/validate_animation_mode.mjs   (headless Chrome + SwiftShader via puppeteer-core; screenshots + JSON -> reports/exp002_1b/)
import puppeteer from 'puppeteer-core';
import { writeFileSync, mkdirSync } from 'node:fs';
const OUT = process.env.OUT || new URL('../reports/exp002_1b', import.meta.url).pathname;
mkdirSync(OUT, { recursive: true });
const APP_URL = process.env.URL || 'http://127.0.0.1:8788/';
const browser = await puppeteer.launch({ headless: 'new', executablePath: process.env.CHROME || '/usr/bin/google-chrome',
  args: ['--no-sandbox', '--enable-unsafe-swiftshader', '--use-gl=angle', '--use-angle=swiftshader-webgl', '--ignore-gpu-blocklist', '--window-size=1280,800'],
  defaultViewport: { width: 1280, height: 800, hasTouch: true } });
const page = await browser.newPage();
const errors = [], logs = [];
page.on('pageerror', e => errors.push('PAGEERROR ' + e.message));
page.on('console', m => { const t = `${m.type()}: ${m.text()}`; logs.push(t); if (m.type() === 'error' && !/404 \(File not found\)/.test(m.text())) errors.push(t); });
page.on('response', r => { if (r.status() >= 400 && !r.url().endsWith('/favicon.ico')) errors.push(`HTTP ${r.status()} ${r.url()}`); });
const sleep = ms => new Promise(r => setTimeout(r, ms));
const t0 = Date.now();
await page.goto(APP_URL, { waitUntil: 'domcontentloaded', timeout: 60000 });
await page.waitForFunction(() => window.__animation?.ready, { timeout: 240000 });
const loadMs = Date.now() - t0;
await sleep(1500);
const st = () => page.evaluate(() => window.__animation.state());
const blur = () => page.evaluate(() => document.activeElement?.blur());
const evidence = { url: page.url(), loadMs, phases: {} };
const NAT = { Walk_Fwd: 1.442, Run_Fwd: 3.016, Walk_Back: 0.647 };

// fps estimate
const fps = await page.evaluate(() => new Promise(res => { let n = 0; const s = performance.now(); const f = () => { n++; if (performance.now() - s < 2000) requestAnimationFrame(f); else res(n / ((performance.now() - s) / 1000)); }; requestAnimationFrame(f); }));
evidence.fps = +fps.toFixed(1);

async function phase(name, fn) {
  await page.evaluate(() => window.__animation.record(true));
  const extra = await fn();
  const rec = await page.evaluate(() => window.__animation.takeRecording());
  evidence.phases[name] = { extra, analysis: analyze(rec), frames: rec.length, start: rec[0] && { x: rec[0].x, z: rec[0].z, t: rec[0].t }, end: rec.at(-1) && { x: rec.at(-1).x, z: rec.at(-1).z, t: rec.at(-1).t, state: rec.at(-1).state, speed: rec.at(-1).speed } };
  evidence.phases[name].trace = rec.filter((_, i) => i % 3 === 0).map(f => ({ t: f.t, x: +f.x.toFixed(3), z: +f.z.toFixed(3), yaw: +(f.yaw * 180 / Math.PI).toFixed(1), v: +f.speed.toFixed(3), st: f.state, clips: f.clips.map(c => `${c.name}:${c.w}@${c.ts}`).join(' ') }));
  if (process.env.RAW) writeFileSync(`${process.env.RAW}/rec_${name}.json`, JSON.stringify(rec));
  console.log(name, JSON.stringify(evidence.phases[name].analysis), '| end', JSON.stringify(evidence.phases[name].end), extra ? '| ' + JSON.stringify(extra) : '');
  return evidence.phases[name];
}

function analyze(rec) {
  const out = { syncFrames: 0, syncMaxErr: 0, measuredVsCtrlMaxErr: 0, maxWeightStep: 0, maxWeightStepAllowed: 0, phaseJumps: 0 };
  const slip = {}; const bones = ['ball_l', 'ball_r', 'foot_l', 'foot_r'];
  const minY = {}; for (const b of bones) minY[b] = Math.min(...rec.map(f => f.feet[b]?.[1] ?? Infinity));
  for (let i = 1; i < rec.length; i++) {
    const a = rec[i - 1], f = rec[i];
    // controller speed vs actual root displacement
    const meas = Math.hypot(f.x - a.x, f.z - a.z) / f.dt;
    if (Math.abs(f.x) < 94 && Math.abs(f.z) < 94) out.measuredVsCtrlMaxErr = Math.max(out.measuredVsCtrlMaxErr, Math.abs(meas - Math.abs(f.speed)));
    // playback-rate sync: dominant locomotion clip timeScale * natural == |speed|
    const dom = f.clips.length === 1 ? f.clips[0] : null;
    if (dom && NAT[dom.name] && dom.w > 0.999) { out.syncFrames++; out.syncMaxErr = Math.max(out.syncMaxErr, Math.abs(Math.abs(dom.ts) * NAT[dom.name] - Math.abs(f.speed))); }
    // crossfade smoothness: per-frame weight change bounded by dt / 0.25 s
    const names = new Set([...a.clips.map(c => c.name), ...f.clips.map(c => c.name)]);
    for (const n of names) {
      const wa = a.clips.find(c => c.name === n)?.w ?? 0, wf = f.clips.find(c => c.name === n)?.w ?? 0;
      out.maxWeightStep = Math.max(out.maxWeightStep, Math.abs(wf - wa));
      // phase continuity while visible: time advances by ts*dt (mod duration) - no restarts
      const ca = a.clips.find(c => c.name === n), cf = f.clips.find(c => c.name === n);
      if (ca && cf && NAT[n]) {
        const dur = { Walk_Fwd: 1.0667, Run_Fwd: 0.7667, Walk_Back: 1.6667 }[n];
        let adv = cf.time - ca.time; adv = ((adv % dur) + dur) % dur; const exp = (((cf.ts * f.dt) % dur) + dur) % dur;
        const d = Math.min(Math.abs(adv - exp), dur - Math.abs(adv - exp));
        if (d > 0.02) out.phaseJumps++;
      }
    }
    out.maxWeightStepAllowed = Math.max(out.maxWeightStepAllowed, f.dt / 0.25 + 1e-6);
    // foot slip: horizontal world speed of a planted (lowest 2 cm) foot bone during single-clip locomotion
    if (dom && NAT[dom.name] && Math.abs(f.speed) > 0.3) for (const b of bones) {
      const pa = a.feet[b], pf = f.feet[b]; if (!pa || !pf) continue;
      if (pa[1] < minY[b] + 0.02 && pf[1] < minY[b] + 0.02) {
        (slip[dom.name] ||= []).push({ v: Math.hypot(pf[0] - pa[0], pf[2] - pa[2]) / f.dt, body: Math.abs(f.speed) });
      }
    }
  }
  out.slip = {};
  for (const [n, arr] of Object.entries(slip)) {
    const vs = arr.map(s => s.v).sort((x, y) => x - y);
    out.slip[n] = { samples: vs.length, medianPlantedFootSpeed: +vs[Math.floor(vs.length / 2)].toFixed(3), meanPlantedFootSpeed: +(vs.reduce((s, v) => s + v, 0) / vs.length).toFixed(3), bodySpeed: +(arr.reduce((s, a) => s + a.body, 0) / arr.length).toFixed(3) };
  }
  for (const k of ['syncMaxErr', 'measuredVsCtrlMaxErr', 'maxWeightStep', 'maxWeightStepAllowed']) out[k] = +out[k].toFixed(4);
  return out;
}

async function hold(keys, ms) { for (const k of keys) await page.keyboard.down(k); await sleep(ms); for (const k of [...keys].reverse()) await page.keyboard.up(k); }
async function waitIdle(ms = 15000) { const end = Date.now() + ms; while (Date.now() < end) { const s = await st(); if (s.locoState === 'Idle' && s.move.speed === 0 && s.anim.clips.find(c => c.name === 'Idle').weight > 0.999) return true; await sleep(250); } return false; }
async function shot(name) { await page.screenshot({ path: `${OUT}/${name}.png` }); return `${OUT}/${name}.png`; }
async function pickCam(name) { await page.click(`button[data-view="${name}"]`); await blur(); await sleep(300); }

// ---- Check 1: clips discovered ----
const clips = await page.evaluate(() => window.__animation.clips);
const selectOpts = await page.$$eval('#anim-debug select option', os => os.map(o => o.value));
const clipListText = await page.$$eval('#anim-debug .ad-clips li', ls => ls.map(l => l.textContent));
evidence.check1 = { clips, selectOpts, clipListText };
console.log('CHECK1', JSON.stringify(selectOpts), clips.map(c => `${c.name}:${c.loop}`).join(','));
await shot('01_idle_debugger_open');

// ---- Checks 2/3: WASD, from the FRONT camera (camera at +Z looking -Z: W = -Z, D = +X) ----
await pickCam('FRONT');
await phase('W_forward', async () => { await hold(['KeyW'], 9000); return { during: null }; });
await shot('02_after_W_walk');
await phase('release_after_W', async () => ({ idle: await waitIdle() }));
await phase('S_backward', async () => { await hold(['KeyS'], 9000); });
await phase('release_after_S', async () => ({ idle: await waitIdle() }));
await phase('A_left', async () => { await hold(['KeyA'], 7000); });
await waitIdle();
await phase('D_right', async () => { await hold(['KeyD'], 7000); });
await waitIdle();
await phase('WD_diagonal', async () => { await hold(['KeyW', 'KeyD'], 7000); });
await waitIdle();

// ---- Check 4: Shift run, Caps Lock toggle ----
await phase('W_shift_run', async () => {
  await page.keyboard.down('KeyW'); await sleep(2500); await page.keyboard.down('Shift'); await sleep(7000);
  const running = await st(); await shot('03_running_shift');
  await page.keyboard.up('Shift'); await sleep(5000); const afterShift = await st(); await page.keyboard.up('KeyW');
  return { whileShift: { loco: running.locoState, speed: running.move.speed, clips: running.anim.clips.filter(c => c.weight > 0.01).map(c => [c.name, +c.weight.toFixed(2), +c.timeScale.toFixed(3)]) },
    afterShiftRelease: { loco: afterShift.locoState, speed: afterShift.move.speed } };
});
await waitIdle();
await phase('capslock_toggle', async () => {
  await page.keyboard.press('CapsLock'); const capsOn = (await st()).run.capsLock;
  await page.keyboard.down('KeyW'); await sleep(8000); const r = await st(); await page.keyboard.up('KeyW');
  await waitIdle();
  await page.keyboard.press('CapsLock'); const capsOff = (await st()).run.capsLock;
  await page.keyboard.down('KeyW'); await sleep(6000); const w = await st(); await page.keyboard.up('KeyW');
  return { capsOn, runWithCaps: { loco: r.locoState, speed: r.move.speed }, capsOff, afterCapsOff: { loco: w.locoState, speed: w.move.speed } };
});
await waitIdle();

// ---- Check 7: a full transition chain Idle -> Walk -> Run -> Walk -> Back -> Idle ----
await phase('transition_chain', async () => {
  await page.keyboard.down('KeyW'); await sleep(3000); await page.keyboard.down('Shift'); await sleep(4000);
  await page.keyboard.up('Shift'); await sleep(3000); await page.keyboard.up('KeyW');
  await page.keyboard.down('KeyS'); await sleep(5000); await page.keyboard.up('KeyS');
  return { idle: await waitIdle() };
});

// ---- Left joystick via TOUCH (and the source readout) ----
const ring = await page.$eval('#human .human-stick[data-side="left"] .human-ring', el => { const r = el.getBoundingClientRect(); return { cx: r.left + r.width / 2, cy: r.top + r.height / 2, R: r.width / 2, bottom: innerHeight - r.bottom }; });
evidence.leftRing = ring;
await phase('touch_left_stick_left', async () => {
  await page.touchscreen.touchStart(ring.cx, ring.cy);
  await page.touchscreen.touchMove(ring.cx - 0.3 * ring.R, ring.cy);
  await page.touchscreen.touchMove(ring.cx - 0.6 * ring.R, ring.cy);
  await sleep(6000); const s = await st();
  await shot('04_touch_left_stick');
  await page.touchscreen.touchMove(ring.cx - 1.3 * ring.R, ring.cy); await sleep(6000); const s2 = await st();
  await page.touchscreen.touchEnd();
  const rel = await st(); const idle = await waitIdle();
  return { partial: { source: s.input.source, vec: [s.input.x, s.input.y], speed: s.move.speed, loco: s.locoState, yaw: s.move.yaw * 180 / Math.PI },
    rim: { source: s2.input.source, vec: [s2.input.x, s2.input.y], gait: s2.input.gait, speed: s2.move.speed, loco: s2.locoState },
    justReleased: { speed: rel.move.speed, source: rel.input.source }, idle };
});
await phase('mouse_left_stick_up', async () => {
  await page.mouse.move(ring.cx, ring.cy); await page.mouse.down(); await page.mouse.move(ring.cx, ring.cy - 0.7 * ring.R, { steps: 4 });
  await sleep(6000); const s = await st(); await page.mouse.up(); const idle = await waitIdle();
  return { source: s.input.source, vec: [s.input.x, s.input.y], speed: s.move.speed, loco: s.locoState, idle };
});
// Legacy right stick (push up) still walks via the same controller.
const rring = await page.$eval('#human .human-stick[data-side="right"] .human-ring', el => { const r = el.getBoundingClientRect(); return { cx: r.left + r.width / 2, cy: r.top + r.height / 2, R: r.width / 2 }; });
await phase('right_stick_legacy', async () => {
  await page.mouse.move(rring.cx, rring.cy); await page.mouse.down(); await page.mouse.move(rring.cx, rring.cy - 0.6 * rring.R, { steps: 4 });
  await sleep(5000); const s = await st(); await page.mouse.up(); await waitIdle();
  return { source: s.input.source, speed: s.move.speed, loco: s.locoState };
});

// ---- Debugger controls: pause / speed slider ----
const ctrl = {};
await page.keyboard.down('KeyW'); await sleep(3000);
await page.click('#anim-debug [data-action="pause"]'); await sleep(1500);
const p1 = await st(); await sleep(2000); const p2 = await st();
ctrl.pause = { paused: p1.anim.paused, posBefore: p1.move.position, posAfter2s: p2.move.position, walkTime1: p1.anim.clips.find(c => c.name === 'Walk_Fwd').time, walkTime2: p2.anim.clips.find(c => c.name === 'Walk_Fwd').time, label: await page.$eval('#anim-debug [data-action="pause"]', b => b.textContent) };
await page.click('#anim-debug [data-action="pause"]'); await sleep(1500);
const p3 = await st(); ctrl.resume = { paused: p3.anim.paused, pos: p3.move.position };
await page.$eval('#anim-debug input[data-action="speed"]', el => { el.value = '0.5'; el.dispatchEvent(new Event('input', { bubbles: true })); });
await sleep(2500); const sp = await st();
ctrl.speedHalf = { speed: sp.anim.speed, ctrlSpeed: sp.move.speed, walkTs: sp.anim.clips.find(c => c.name === 'Walk_Fwd').timeScale, label: await page.$eval('#anim-debug .ad-slider span', s => s.textContent) };
await page.$eval('#anim-debug input[data-action="speed"]', el => { el.value = '1'; el.dispatchEvent(new Event('input', { bubbles: true })); });
await page.keyboard.up('KeyW'); await blur(); await waitIdle();
evidence.debuggerControls = ctrl;
console.log('DEBUGGER CONTROLS', JSON.stringify(ctrl));
const liveFields = await page.$$eval('#anim-debug .ad-live dt', d => d.map(x => x.textContent));
evidence.liveFields = liveFields;

// ---- Check 8: manual preview incl. one-shots, through the debugger UI ----
const preview = {};
await page.click('#anim-debug [data-action="mode"]'); await sleep(500);
preview.modeAfterToggle = (await st()).anim.mode;
for (const clip of ['Squat', 'ToPlank', 'Jump', 'Walk_Back']) {
  await page.select('#anim-debug select', clip); await page.click('#anim-debug [data-action="preview"]'); await blur();
  const dur = clips.find(c => c.name === clip).duration;
  await sleep(1500); const mid = await st();
  // wait (sim time) past the clip end
  const startT = mid.simTime; let s;
  const end = Date.now() + 60000;
  do { await sleep(500); s = await st(); } while (s.simTime - startT < dur + 0.8 && Date.now() < end);
  const c = s.anim.clips.find(x => x.name === clip);
  preview[clip] = { state: s.anim.state, previewClip: s.anim.previewClip, weight: +c.weight.toFixed(3), time: +c.time.toFixed(3), duration: +dur.toFixed(3), finished: c.finished, loop: c.loop, midTime: +mid.anim.clips.find(x => x.name === clip).time.toFixed(3), activeField: await page.$eval('#anim-debug dd[data-field="active"]', d => d.textContent), pos: s.move.position };
  if (clip === 'ToPlank') { await sleep(1500); preview[clip].timeAfterHold = +(await st()).anim.clips.find(x => x.name === clip).time.toFixed(3); await shot('05_preview_ToPlank_hold'); }
  console.log('PREVIEW', clip, JSON.stringify(preview[clip]));
}
// Movement input is ignored in Manual mode.
await hold(['KeyW'], 2000); preview.manualIgnoresInput = (await st()).move.speed === 0;
await page.click('#anim-debug [data-action="mode"]'); await blur(); await sleep(500);
preview.backToAuto = (await st()).anim.mode;
preview.idleAfterAuto = await waitIdle();
evidence.check8 = preview;

// ---- Check 9: facial morph inspector ----
await page.click('#anim-debug .ad-face summary'); await sleep(300);
const face = {};
face.summary = await page.$eval('#anim-debug .ad-face summary', s => s.textContent);
face.sliderCount = await page.$$eval('#anim-debug .ad-morph input[type=range]', l => l.length);
face.names = await page.$$eval('#anim-debug .ad-morph input[type=range]', l => l.map(i => i.dataset.morph));
// Drive every slider through the UI and read the actual morphTargetInfluences on all 6 head primitives.
face.perMorph = await page.evaluate(async () => {
  const res = [];
  const head = []; window.__animation; // meshes via getMorph only reads the first; check all via scene traversal not exposed -> use getMorph + count
  for (const input of document.querySelectorAll('#anim-debug .ad-morph input[type=range]')) {
    input.value = '0.8'; input.dispatchEvent(new Event('input', { bubbles: true }));
    res.push([input.dataset.morph, window.__animation.getMorph(input.dataset.morph)]);
    input.value = '0'; input.dispatchEvent(new Event('input', { bubbles: true }));
  }
  return res;
});
face.allControllable = face.perMorph.every(([, v]) => Math.abs(v - 0.8) < 1e-6);
// Independence from body animation: set a face while walking; it must persist and not be touched by the mixer.
for (const [n, v] of [['jawOpen', 0.6], ['mouthSmileLeft', 1], ['mouthSmileLeftold', 0], ['eyeBlinkLeft', 1], ['browInnerUp', 0.7]]) {
  await page.$eval(`#anim-debug input[data-morph="${n}"]`, (el, v) => { el.value = String(v); el.dispatchEvent(new Event('input', { bubbles: true })); }, v);
}
await blur();
await page.keyboard.down('KeyW'); await sleep(4000);
face.whileWalking = await page.evaluate(() => ({ loco: window.__animation.state().locoState, jawOpen: window.__animation.getMorph('jawOpen'), eyeBlinkLeft: window.__animation.getMorph('eyeBlinkLeft'), browInnerUp: window.__animation.getMorph('browInnerUp') }));
await page.keyboard.up('KeyW'); await waitIdle();
const sCam = await st();
const P = sCam.move.position, yaw = sCam.move.yaw;
// close-up of the face from the front of the character
await page.evaluate((x, z, yaw) => window.__animation.view([x + Math.sin(yaw) * 0.85, 1.62, z + Math.cos(yaw) * 0.85], [x, 1.55, z]), P.x, P.z, yaw);
await page.$eval('#anim-debug .ad-face summary', el => el.scrollIntoView({ block: 'start' }));
await sleep(1500); await shot('06_face_inspector_morphs');
await page.click('#anim-debug [data-action="reset-face"]'); await sleep(300);
face.afterReset = await page.evaluate(() => window.__animation.morphs.names.every(n => window.__animation.getMorph(n) === 0));
face.slidersAfterReset = await page.$$eval('#anim-debug .ad-morph input[type=range]', l => l.every(i => Number(i.value) === 0));
await sleep(800); await shot('07_face_after_reset');
evidence.check9 = { ...face, perMorph: undefined, names: face.names };
console.log('FACE', JSON.stringify({ summary: face.summary, sliderCount: face.sliderCount, allControllable: face.allControllable, whileWalking: face.whileWalking, afterReset: face.afterReset, slidersAfterReset: face.slidersAfterReset }));
await page.click('#anim-debug .ad-face summary');
await page.click('#anim-debug input[data-action="follow"]'); await page.click('#anim-debug input[data-action="follow"]'); // follow back on (view() turned it off)

// ---- Check 10 (Animation part): camera orbit by mouse and touch, presets, panel isolation ----
await pickCam('RESET');
const cam0 = (await st()).camera;
await page.mouse.move(800, 300); await page.mouse.down(); await page.mouse.move(900, 330, { steps: 6 }); await page.mouse.up(); await sleep(400);
const cam1 = (await st()).camera;
await page.touchscreen.touchStart(800, 250); await page.touchscreen.touchMove(860, 250); await page.touchscreen.touchMove(920, 260); await page.touchscreen.touchEnd(); await sleep(400);
const cam2 = (await st()).camera;
await pickCam('FRONT'); const cam3 = (await st()).camera;
// touch drag INSIDE the debugger panel must not orbit the camera nor move Xandra
// A point inside the VISIBLE part of the panel (the body may be scrolled after the Face section).
const panel = await page.$eval('#anim-debug .ad-body', el => { const r = el.getBoundingClientRect(); return { x: r.left + 40, y: r.top + 30 }; });
await page.touchscreen.touchStart(panel.x, panel.y); await page.touchscreen.touchMove(panel.x + 60, panel.y + 5); await page.touchscreen.touchEnd(); await sleep(400);
const cam4 = (await st()).camera; const s4 = await st();
const dist = (a, b) => Math.hypot(...a.map((v, i) => v - b[i]));
evidence.check10anim = { mouseOrbitMoved: +dist(cam0.position, cam1.position).toFixed(3), touchOrbitMoved: +dist(cam1.position, cam2.position).toFixed(3), frontPreset: cam3, panelDragCamDelta: +dist(cam3.position, cam4.position).toFixed(4), panelDragSpeed: s4.move.speed, leftRingBottomPx: ring.bottom };
console.log('CAMERA', JSON.stringify(evidence.check10anim));
// collapse / expand the debugger
await page.click('#anim-debug .ad-toggle'); const collapsed = await page.$eval('#anim-debug', e => e.dataset.open);
await shot('08_debugger_collapsed'); await page.click('#anim-debug .ad-toggle'); const expanded = await page.$eval('#anim-debug', e => e.dataset.open);
evidence.collapse = { collapsed, expanded };
// final: character far from origin with debugger open
await pickCam('RESET'); await sleep(500); await shot('09_moved_debugger_open');

// ---- Mobile layout: portrait + landscape (touch) ----
for (const [name, vp] of [['10_mobile_portrait', { width: 390, height: 844 }], ['11_mobile_landscape', { width: 844, height: 390 }]]) {
  await page.setViewport({ ...vp, hasTouch: true, isMobile: true, deviceScaleFactor: 2 }); await sleep(2500);
  const lay = await page.evaluate(() => {
    const r = s => document.querySelector(s)?.getBoundingClientRect();
    const L = r('#human .human-stick[data-side="left"] .human-ring'), R = r('#human .human-stick[data-side="right"] .human-ring'), D = r('#anim-debug'), B = r('#human .human-status');
    return { vh: innerHeight, vw: innerWidth, leftRing: L && [L.left, L.top, L.right, L.bottom], rightRing: R && [R.left, R.top, R.right, R.bottom], debug: D && [D.left, D.top, D.right, D.bottom], debugOpen: document.querySelector('#anim-debug').dataset.open, status: B && [B.top, B.bottom] };
  });
  evidence[name] = lay; console.log(name, JSON.stringify(lay));
  await shot(name);
  if (lay.debugOpen === '0') {
    await page.tap('#anim-debug .ad-toggle'); await sleep(1200);
    const open = await page.evaluate(() => { const D = document.querySelector('#anim-debug').getBoundingClientRect(); const L = document.querySelector('#human .human-stick[data-side="left"]').getBoundingClientRect(); const R = document.querySelector('#human .human-stick[data-side="right"]').getBoundingClientRect();
      const ov = (a, b) => !(a.right <= b.left || b.right <= a.left || a.bottom <= b.top || b.bottom <= a.top);
      return { debug: [D.left, D.top, D.right, D.bottom], overlapsLeftStick: ov(D, L), overlapsRightStick: ov(D, R) }; });
    evidence[name + '_open'] = open; console.log(name + '_open', JSON.stringify(open));
    await shot(name + '_debugger_open');
    await page.tap('#anim-debug .ad-toggle'); await sleep(300);
  }
}
await page.setViewport({ width: 1280, height: 800, hasTouch: true });

evidence.errors = errors;
evidence.logs = logs.filter(l => l.includes('[animation]'));
writeFileSync(`${OUT}/validation_evidence.json`, JSON.stringify(evidence, null, 1));
console.log('FPS', evidence.fps, 'LOAD', loadMs, 'ERRORS', JSON.stringify(errors));
await browser.close();
