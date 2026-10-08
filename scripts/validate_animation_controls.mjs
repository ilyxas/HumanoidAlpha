// EXP002.1-C validation against the REAL running app. Start it first: ./START.command animation --no-open
// Then: node scripts/validate_animation_controls.mjs   (headless Chrome + SwiftShader via puppeteer-core;
// screenshots + JSON -> reports/exp002_1c/, raw per-frame recordings -> RAW). ONLY=wasd,feedback,... runs a subset.
import puppeteer from 'puppeteer-core';
import { writeFileSync, mkdirSync } from 'node:fs';
const OUT = process.env.OUT || new URL('../reports/exp002_1c', import.meta.url).pathname;
const RAW = process.env.RAW || '/tmp/exp002_1c_raw';
mkdirSync(OUT, { recursive: true }); mkdirSync(RAW, { recursive: true });
const APP_URL = process.env.URL || 'http://127.0.0.1:8788/';
const ONLY = process.env.ONLY ? new Set(process.env.ONLY.split(',')) : null;
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
const evidence = { url: page.url(), loadMs: Date.now() - t0, checks: {} };
await sleep(1500);
const fps = await page.evaluate(() => new Promise(res => { let n = 0; const s = performance.now(); const f = () => { n++; if (performance.now() - s < 2000) requestAnimationFrame(f); else res(n / ((performance.now() - s) / 1000)); }; requestAnimationFrame(f); }));
evidence.fps = +fps.toFixed(1);

const D = r => r * 180 / Math.PI;
const wrap = a => Math.atan2(Math.sin(a), Math.cos(a));
const r3 = v => (v == null ? v : +(+v).toFixed(3));
const r1 = v => +(+v).toFixed(1);
const st = () => page.evaluate(() => window.__animation.state());
const blur = () => page.evaluate(() => document.activeElement?.blur());
async function shot(name) { await page.screenshot({ path: `${OUT}/${name}.png` }); return `${OUT}/${name}.png`; }
async function pickCam(name) { await page.click(`button[data-view="${name}"]`); await blur(); await sleep(400); }
async function waitIdle(ms = 20000) { const end = Date.now() + ms; while (Date.now() < end) { const s = await st(); if (s.locoState === 'Idle' && s.move.speed === 0 && !s.anim.oneShot && (s.anim.clips.find(c => c.name === 'Idle').weight > 0.999)) return true; await sleep(250); } return false; }
async function waitSim(simS, maxMs = 60000) { const a = (await st()).simTime; const end = Date.now() + maxMs; while (Date.now() < end) { await sleep(200); if ((await st()).simTime - a >= simS) return; } }
async function record(fn) { await page.evaluate(() => window.__animation.record(true)); const extra = await fn(); const rec = await page.evaluate(() => window.__animation.takeRecording()); return { rec, extra }; }
const camYawOf = s => s.camera.azimuth + Math.PI; // camera looks toward the target
function expectedYaw(camYaw, ix, iy) { // input (x right, y forward) relative to camera forward on XZ
  const f = [Math.sin(camYaw), Math.cos(camYaw)], right = [-f[1], f[0]];
  return Math.atan2(f[0] * iy + right[0] * ix, f[1] * iy + right[1] * ix);
}
const KEYVEC = { KeyW: [0, 1], KeyS: [0, -1], KeyA: [-1, 0], KeyD: [1, 0] };
function keysVec(keys) { let x = 0, y = 0; for (const k of keys) { x += KEYVEC[k][0]; y += KEYVEC[k][1]; } const m = Math.hypot(x, y); return [x / m, y / m]; }
// Generic smoothness metrics over a recording.
function smooth(rec) {
  const o = { frames: rec.length, maxWeightStep: 0, maxWeightStepAllowed: 0, maxYawRate: 0, walkBackFrames: 0, rootYMax: 0, states: [] };
  for (let i = 0; i < rec.length; i++) {
    const f = rec[i]; o.rootYMax = Math.max(o.rootYMax, Math.abs(f.rootY ?? 0));
    if (f.clips.some(c => c.name === 'Walk_Back' && c.w > 0)) o.walkBackFrames++;
    if (o.states.at(-1) !== f.state) o.states.push(f.state);
    if (!i) continue; const a = rec[i - 1];
    const names = new Set([...a.clips.map(c => c.name), ...f.clips.map(c => c.name)]);
    for (const n of names) o.maxWeightStep = Math.max(o.maxWeightStep, Math.abs((f.clips.find(c => c.name === n)?.w ?? 0) - (a.clips.find(c => c.name === n)?.w ?? 0)));
    o.maxWeightStepAllowed = Math.max(o.maxWeightStepAllowed, f.dt / 0.25 + 1e-3);
    o.maxYawRate = Math.max(o.maxYawRate, Math.abs(wrap(f.yaw - a.yaw)) / f.dt);
  }
  for (const k of ['maxWeightStep', 'maxWeightStepAllowed', 'maxYawRate', 'rootYMax']) o[k] = r3(o[k]);
  return o;
}
// Directional move analysis: heading after the turn, path direction, straightness, camera behaviour.
function moveAnalysis(rec, expYaw) {
  const moving = rec.filter(f => f.input);
  const settledFrom = moving.findIndex(f => Math.abs(wrap(f.yaw - expYaw)) < 0.02);
  const after = settledFrom >= 0 ? moving.slice(settledFrom) : [];
  const a = after[0], b = after.at(-1);
  let pathYaw = null, maxLateral = 0, headingDrift = 0;
  if (a && b && Math.hypot(b.x - a.x, b.z - a.z) > 0.3) {
    pathYaw = Math.atan2(b.x - a.x, b.z - a.z);
    const ux = Math.sin(expYaw), uz = Math.cos(expYaw);
    for (const f of after) { maxLateral = Math.max(maxLateral, Math.abs((f.x - a.x) * uz - (f.z - a.z) * ux)); headingDrift = Math.max(headingDrift, Math.abs(wrap(f.yaw - expYaw))); }
  }
  const camDist = rec.map(f => f.camDist), camY = rec.map(f => f.camY);
  const err = rec.map(f => wrap(f.yaw + Math.PI - f.camAz));
  return {
    expectedYawDeg: r1(D(expYaw)), finalYawDeg: r1(D(moving.at(-1).yaw)), finalYawErrDeg: r3(D(Math.abs(wrap(moving.at(-1).yaw - expYaw)))),
    yawChangeAfterReleaseDeg: r3(D(Math.abs(wrap(rec.at(-1).yaw - moving.at(-1).yaw)))),
    turnSimS: a ? r3(a.t - moving[0].t) : null, pathYawErrDeg: pathYaw == null ? null : r3(D(Math.abs(wrap(pathYaw - expYaw)))),
    straightPathM: a && b ? r3(Math.hypot(b.x - a.x, b.z - a.z)) : 0, maxLateralDevM: r3(maxLateral), headingDriftAfterTurnDeg: r3(D(headingDrift)),
    refYawChanges: new Set(moving.map(f => f.refYaw.toFixed(4))).size, refReasons: [...new Set(moving.map(f => f.ref))],
    camDistRangeM: r3(Math.max(...camDist) - Math.min(...camDist)), camHeightRangeM: r3(Math.max(...camY) - Math.min(...camY)),
    camErrStartDeg: r1(D(err[0])), camErrEndDeg: r1(D(err.at(-1))), peakSpeed: r3(Math.max(...rec.map(f => Math.abs(f.speed)))),
    clipsSeen: [...new Set(rec.flatMap(f => f.clips.filter(c => c.w > 0.5).map(c => c.name)))],
  };
}
// Spring analysis for the auto-align: convergence time, overshoot (sign change of the error), rate.
function alignAnalysis(rec) {
  const err = rec.map(f => wrap(f.yaw + Math.PI - f.camAz));
  const s0 = Math.sign(err.find(e => Math.abs(e) > 0.05) || 1);
  let overshoot = 0, lastBig = null, maxRate = 0, peak = 0, signChanges = 0;
  const first = rec.findIndex(f => f.input);
  for (let i = 0; i < rec.length; i++) {
    if (Math.sign(err[i]) === -s0 && Math.abs(err[i]) > 1e-3) overshoot = Math.max(overshoot, Math.abs(err[i]));
    if (rec[i].input && Math.abs(err[i]) >= 5 * Math.PI / 180) lastBig = i;
    peak = Math.max(peak, Math.abs(err[i]));
    if (i) { maxRate = Math.max(maxRate, Math.abs(wrap(rec[i].camAz - rec[i - 1].camAz)) / rec[i].dt); if (Math.abs(err[i]) > 0.01 && Math.abs(err[i - 1]) > 0.01 && Math.sign(err[i]) !== Math.sign(err[i - 1])) signChanges++; }
  }
  return { peakErrDeg: r1(D(peak)), errEndDeg: r1(D(err.at(-1))), overshootDeg: r3(D(overshoot)), errSignChanges: signChanges, settleBelow5degSimS: lastBig == null ? 0 : r3(rec[Math.min(lastBig + 1, rec.length - 1)].t - rec[first].t), maxCamYawRateDegS: r1(D(maxRate)), statuses: [...new Set(rec.map(f => f.align))] };
}
const want = n => !ONLY || ONLY.has(n);
const save = (name, rec) => writeFileSync(`${RAW}/rec_${name}.json`, JSON.stringify(rec));

// Put Xandra in a known place/orientation and the camera on a preset.
async function setup(yaw, cam) { await waitIdle(); await page.evaluate(y => window.__animation.setYaw(y), yaw); await sleep(300); await pickCam(cam); await sleep(300); }
async function moveTest(name, keys, ms = 5500) {
  await waitIdle();
  const s0 = await st(); const [ix, iy] = keysVec(keys); const exp = expectedYaw(camYawOf(s0), ix, iy);
  const { rec } = await record(async () => { for (const k of keys) await page.keyboard.down(k); await sleep(ms); for (const k of keys) await page.keyboard.up(k); await sleep(400); });
  save(name, rec);
  const r = { keys: keys.join('+'), startYawDeg: r1(D(s0.move.yaw)), cameraYawDeg: r1(D(camYawOf(s0))), ...moveAnalysis(rec, exp), smooth: smooth(rec) };
  console.log('MOVE', name, JSON.stringify(r)); return r;
}

// ===== 1. WASD + 4 diagonals from three camera angles =====
if (want('wasd')) {
  const W = {};
  // Angle A: camera behind her (BACK preset), she faces +Z.
  for (const [n, keys] of [['W', ['KeyW']], ['S', ['KeyS']], ['A', ['KeyA']], ['D', ['KeyD']]]) { await setup(0, 'BACK'); W[`camBack_${n}`] = await moveTest(`camBack_${n}`, keys); }
  // Angle B: she is turned 57 deg, camera on her RIGHT side.
  for (const [n, keys] of [['W', ['KeyW']], ['WA', ['KeyW', 'KeyA']], ['WD', ['KeyW', 'KeyD']]]) { await setup(1.0, 'RIGHT'); W[`camRight_${n}`] = await moveTest(`camRight_${n}`, keys); }
  // Angle C: camera in FRONT of her, then orbited by a real mouse drag (~ -60 deg).
  for (const [n, keys] of [['SA', ['KeyS', 'KeyA']], ['SD', ['KeyS', 'KeyD']], ['D', ['KeyD']]]) {
    await setup(-2.2, 'FRONT');
    await page.mouse.move(500, 400); await page.mouse.down(); await page.mouse.move(580, 400, { steps: 8 }); await page.mouse.up(); await sleep(300);
    W[`camOrbit_${n}`] = await moveTest(`camOrbit_${n}`, keys);
  }
  await setup(0.6, 'RESET'); W.camReset_WD = await moveTest('camReset_WD', ['KeyW', 'KeyD']);
  evidence.checks.wasd = W;
  await waitIdle();
}

// ===== 2. No feedback loop: hold D for a long time with auto-align on =====
if (want('feedback')) {
  await setup(0, 'BACK');
  const s0 = await st(); const exp = expectedYaw(camYawOf(s0), 1, 0);
  let mid;
  const { rec } = await record(async () => { await page.keyboard.down('KeyD'); await sleep(7000); mid = await shot('c01_hold_D_camera_behind'); await sleep(7000); await page.keyboard.up('KeyD'); await sleep(300); });
  save('feedback_D', rec);
  evidence.checks.feedback = { ...moveAnalysis(rec, exp), align: alignAnalysis(rec), smooth: smooth(rec), screenshot: mid };
  console.log('FEEDBACK', JSON.stringify(evidence.checks.feedback));
  await waitIdle();
}

// ===== 3. S turns her around; camera swings behind her (biggest auto-align case) =====
if (want('turnaround')) {
  await setup(0, 'BACK');
  const s0 = await st(); const exp = expectedYaw(camYawOf(s0), 0, -1);
  const shots = [];
  const { rec } = await record(async () => { await page.keyboard.down('KeyS'); await sleep(1600); shots.push(await shot('c02_S_turning_around')); await sleep(9000); shots.push(await shot('c03_S_walking_camera_behind')); await page.keyboard.up('KeyS'); await sleep(2500); });
  save('turnaround_S', rec);
  const stopIdx = rec.findIndex((f, i) => i && !f.input && rec[i - 1].input);
  const azAtStop = rec[stopIdx]?.camAz, azEnd = rec.at(-1).camAz;
  evidence.checks.turnaround = { ...moveAnalysis(rec, exp), align: alignAnalysis(rec), smooth: smooth(rec), camYawAfterStopDriftDeg: stopIdx > 0 ? r3(D(Math.abs(wrap(azEnd - azAtStop)))) : null, screenshots: shots };
  console.log('TURNAROUND', JSON.stringify(evidence.checks.turnaround));
  await waitIdle();
}

// ===== 4. Camera: stationary free orbit; manual override while moving; resume =====
if (want('camera')) {
  await setup(0.4, 'BACK');
  const c = {};
  const a = await st();
  await page.mouse.move(500, 400); await page.mouse.down(); await page.mouse.move(640, 380, { steps: 10 }); await page.mouse.up(); await sleep(1500);
  const b = await st();
  const centered = s => Math.hypot(s.camera.target[0] - s.move.position.x, s.camera.target[2] - s.move.position.z);
  c.stationaryOrbit = { camAzChangeDeg: r1(D(wrap(b.camera.azimuth - a.camera.azimuth))), characterYawChangeDeg: r3(D(wrap(b.move.yaw - a.move.yaw))), speedAfter: b.move.speed, targetOffsetFromHerM: r3(centered(b)), distBefore: r3(a.camera.distance), distAfter: r3(b.camera.distance), status: b.camera.status };
  await shot('c04_stationary_orbit');
  // Manual override during movement.
  const { rec } = await record(async () => {
    await page.keyboard.down('KeyW'); await sleep(2500);
    await page.keyboard.down('KeyD'); await page.keyboard.up('KeyW'); await sleep(3500); // turn 90 deg: auto-align starts
    await page.mouse.move(500, 400); await page.mouse.down();
    for (let i = 1; i <= 10; i++) { await page.mouse.move(500 - 14 * i, 400); await sleep(180); }
    c.duringDrag = await st(); await shot('c05_manual_orbit_while_moving');
    await sleep(1200); // hold the drag still: auto-align must not fight it
    c.dragHeld = await st();
    await page.mouse.up(); c.afterRelease = await st();
    await sleep(6000); c.resumed = await st();
    await page.keyboard.up('KeyD'); await sleep(2500); c.stopped = await st(); await sleep(1500); c.stopped2 = await st();
  });
  save('camera_override', rec);
  const pick = s => ({ status: s.camera.status, azDeg: r1(D(s.camera.azimuth)), errDeg: r1(D(wrap(s.move.yaw + Math.PI - s.camera.azimuth))), yawDeg: r1(D(s.move.yaw)), dist: r3(s.camera.distance), refReason: s.ref.reason, speed: r3(s.move.speed) });
  for (const k of ['duringDrag', 'dragHeld', 'afterRelease', 'resumed', 'stopped', 'stopped2']) c[k] = pick(c[k]);
  // Frames while the button was held: auto-align must not rotate the camera on its own.
  const manual = rec.filter(f => f.align === 'manual orbit');
  const autoDuringManual = manual.slice(1).reduce((m, f) => Math.max(m, Math.abs(f.omega)), 0);
  const resumeFrames = rec.filter(f => f.align === 'resuming');
  c.override = { manualFrames: manual.length, maxAutoOmegaDuringManual: r3(autoDuringManual), resumingFrames: resumeFrames.length, statuses: [...new Set(rec.map(f => f.align))], refReasonsSeen: [...new Set(rec.map(f => f.ref))], camDistRangeM: r3(Math.max(...rec.map(f => f.camDist)) - Math.min(...rec.map(f => f.camDist))), camHeightRangeM: r3(Math.max(...rec.map(f => f.camY)) - Math.min(...rec.map(f => f.camY))) };
  c.camYawDriftAfterStopDeg = r3(D(Math.abs(wrap(rec.at(-1).camAz - (c.stoppedAzRaw = rec.findLast(f => f.input)?.camAz ?? 0)))));
  // Touch orbit (one finger outside the sticks) still orbits.
  const t1 = await st(); await page.touchscreen.touchStart(700, 300); await page.touchscreen.touchMove(760, 300); await page.touchscreen.touchMove(820, 305); await page.touchscreen.touchEnd(); await sleep(600);
  const t2 = await st(); c.touchOrbitAzChangeDeg = r1(D(wrap(t2.camera.azimuth - t1.camera.azimuth)));
  // LOCK disables auto-align; auto-align checkbox off disables it.
  await page.click('#lock'); await blur();
  const { rec: lockRec } = await record(async () => { await page.keyboard.down('KeyA'); await sleep(4000); await page.keyboard.up('KeyA'); });
  await page.click('#lock'); await blur();
  c.locked = { statuses: [...new Set(lockRec.map(f => f.align))], camAzRangeDeg: r3(D(Math.max(...lockRec.map(f => f.camAz)) - Math.min(...lockRec.map(f => f.camAz)))) };
  evidence.checks.camera = c;
  console.log('CAMERA', JSON.stringify(c));
  await waitIdle();
}

// ===== 5. Joystick: low and high magnitude, camera-relative =====
if (want('joystick')) {
  await setup(0, 'BACK');
  const ring = await page.$eval('#human .human-stick[data-side="left"] .human-ring', el => { const r = el.getBoundingClientRect(); return { cx: r.left + r.width / 2, cy: r.top + r.height / 2, R: r.width / 2 }; });
  const j = {};
  for (const [name, dx, dy] of [['low_right', 0.35, 0], ['mid_up', 0, -0.6], ['high_rim_left', -1.3, 0], ['diag_up_left', -0.7, -0.7]]) {
    await setup(0, 'BACK');
    const s0 = await st();
    let during;
    const { rec } = await record(async () => {
      await page.touchscreen.touchStart(ring.cx, ring.cy); await page.touchscreen.touchMove(ring.cx + dx * ring.R * 0.5, ring.cy + dy * ring.R * 0.5); await page.touchscreen.touchMove(ring.cx + dx * ring.R, ring.cy + dy * ring.R);
      await sleep(5500); during = await st(); if (name === 'high_rim_left') await shot('c06_joystick_rim_run');
      await page.touchscreen.touchEnd(); await sleep(300);
    });
    save(`joy_${name}`, rec);
    const exp = expectedYaw(camYawOf(s0), during.input.x, during.input.y);
    j[name] = { source: during.input.source, vec: [r3(during.input.x), r3(during.input.y)], mag: r3(Math.hypot(during.input.x, during.input.y)), gait: r3(during.input.gait), speed: r3(during.move.speed), targetSpeed: r3(during.move.targetSpeed), loco: during.locoState, ...moveAnalysis(rec, exp), smooth: smooth(rec) };
    console.log('JOY', name, JSON.stringify(j[name]));
  }
  // Mouse on the left stick also works.
  await setup(0, 'BACK');
  await page.mouse.move(ring.cx, ring.cy); await page.mouse.down(); await page.mouse.move(ring.cx + 0.6 * ring.R, ring.cy, { steps: 4 }); await sleep(4000);
  const m = await st(); await page.mouse.up();
  j.mouse_right = { source: m.input.source, vec: [r3(m.input.x), r3(m.input.y)], speed: r3(m.move.speed), yawDeg: r1(D(m.move.yaw)) };
  evidence.checks.joystick = j;
  await waitIdle();
}

// ===== 6. Shift & Caps Lock =====
if (want('run')) {
  const r = {};
  await setup(0, 'BACK');
  let { rec } = await record(async () => { await page.keyboard.down('KeyA'); await sleep(2500); await page.keyboard.down('Shift'); await sleep(6000); r.shift = await st(); await shot('c07_shift_run_left'); await page.keyboard.up('Shift'); await sleep(4000); r.afterShift = await st(); await page.keyboard.up('KeyA'); await sleep(300); });
  save('run_shift', rec); r.shiftSmooth = smooth(rec);
  await setup(0, 'BACK');
  await page.keyboard.press('CapsLock'); r.capsOn = (await st()).run.capsLock;
  ({ rec } = await record(async () => { await page.keyboard.down('KeyS'); await sleep(7000); r.caps = await st(); await page.keyboard.up('KeyS'); await sleep(300); }));
  save('run_caps', rec); r.capsSmooth = smooth(rec);
  await page.keyboard.press('CapsLock'); r.capsOff = (await st()).run.capsLock;
  const p = s => ({ loco: s.locoState, speed: r3(s.move.speed), clips: s.anim.clips.filter(c => c.weight > 0.01).map(c => [c.name, r3(c.weight), r3(c.timeScale)]), yawDeg: r1(D(s.move.yaw)) });
  for (const k of ['shift', 'afterShift', 'caps']) r[k] = p(r[k]);
  evidence.checks.run = r; console.log('RUN', JSON.stringify(r));
  await waitIdle();
}

// ===== 7. Presets relative to her position AND facing, after translation + rotation =====
if (want('presets')) {
  await setup(0, 'BACK');
  await page.keyboard.down('KeyW'); await page.keyboard.down('KeyD'); await sleep(5000); await page.keyboard.up('KeyW'); await sleep(4000); await page.keyboard.up('KeyD');
  await waitIdle();
  const OFF = { FRONT: [0, 1.6, 5.5], LEFT: [5.5, 1.6, 0], BACK: [0, 1.6, -5.5], RIGHT: [-5.5, 1.6, 0], TOP: [0, 6.2, 0], RESET: [3.6, 2.5, 4.6] };
  const P = {};
  for (const name of ['FRONT', 'BACK', 'LEFT', 'RIGHT', 'TOP', 'RESET']) {
    await pickCam(name); const s = await st();
    const yaw = s.move.yaw, px = s.move.position.x, pz = s.move.position.z;
    const o = OFF[name], ex = [px + o[0] * Math.cos(yaw) + o[2] * Math.sin(yaw), o[1], pz - o[0] * Math.sin(yaw) + o[2] * Math.cos(yaw)];
    const posErr = Math.hypot(...s.camera.position.map((v, i) => v - ex[i]));
    const tgtErr = Math.hypot(s.camera.target[0] - px, s.camera.target[1] - 0.7, s.camera.target[2] - pz);
    // Where does the camera sit in HER frame? (angle of camera around her, 0 = in front)
    const rel = wrap(Math.atan2(s.camera.position[0] - px, s.camera.position[2] - pz) - yaw);
    P[name] = { herPos: [r3(px), r3(pz)], herYawDeg: r1(D(yaw)), camPosErrM: r3(posErr), targetErrM: r3(tgtErr), cameraAngleInHerFrameDeg: name === 'TOP' ? null : r1(D(rel)) };
    if (name === 'FRONT') P[name].screenshot = await shot('c08_preset_FRONT_after_move_rotate');
    if (name === 'BACK') P[name].screenshot = await shot('c09_preset_BACK_after_move_rotate');
    if (name === 'LEFT') P[name].screenshot = await shot('c10_preset_LEFT_after_move_rotate');
  }
  evidence.checks.presets = P; console.log('PRESETS', JSON.stringify(P));
}

// ===== 8. One-shots Space/1/2/3 =====
if (want('oneshots')) {
  const O = {};
  const clipDur = Object.fromEntries((await page.evaluate(() => window.__animation.clips)).map(c => [c.name, c.duration]));
  async function oneShot(key, clip, extra) {
    await setup(0.9, 'RESET'); await waitIdle();
    const s0 = await st(); const shots = [];
    const { rec, extra: ex } = await record(async () => {
      await page.keyboard.down(key);
      for (let i = 0; i < 6; i++) { await sleep(120); await page.keyboard.down(key); } // OS auto-repeat (repeat: true)
      await page.keyboard.up(key);
      const r = extra ? await extra(shots) : null;
      const end = Date.now() + 90000; let s;
      do { await sleep(300); s = await st(); } while ((s.anim.oneShot || s.anim.clips.find(c => c.name === clip).weight > 0 || s.anim.clips.find(c => c.name === 'Idle').weight < 0.999) && Date.now() < end && !(r && r.stopEarly));
      await sleep(500);
      return r;
    });
    save(`oneshot_${key}`, rec);
    const on = rec.filter(f => f.oneShot && f.oneShot.startsWith(clip));
    const started = rec.filter(f => f.clips.some(c => c.name === clip && c.w > 0));
    const triggers = logs.filter(l => l.includes('[animation] one-shot')).slice(-12);
    const xs = rec.map(f => f.x), zs = rec.map(f => f.z), py = rec.map(f => f.pelvisY).filter(v => v != null);
    const phases = []; for (const f of rec) { const p = f.oneShot || (f.clips.some(c => c.name === clip && c.w > 0) ? 'returning' : f.state); if (phases.at(-1) !== p) phases.push(p); }
    const res = {
      key, clip, clipDurationS: r3(clipDur[clip]), playSimS: on.length ? r3(on.filter(f => f.oneShot.endsWith('play')).length && (on.filter(f => f.oneShot.endsWith('play')).at(-1).t - on[0].t)) : 0,
      holdSimS: r3((on.filter(f => f.oneShot.endsWith('hold')).at(-1)?.t ?? 0) - (on.find(f => f.oneShot.endsWith('hold'))?.t ?? 0)),
      visibleSimS: started.length ? r3(started.at(-1).t - started[0].t) : 0, phases,
      maxWeight: r3(Math.max(0, ...rec.map(f => f.clips.find(c => c.name === clip)?.w ?? 0))),
      rootXZDriftM: r3(Math.hypot(Math.max(...xs) - Math.min(...xs), Math.max(...zs) - Math.min(...zs))), rootYMax: r3(Math.max(...rec.map(f => Math.abs(f.rootY)))),
      pelvisYMin: r3(Math.min(...py)), pelvisYMax: r3(Math.max(...py)), endState: rec.at(-1).state, endClips: rec.at(-1).clips.map(c => `${c.name}:${c.w}`),
      restarts: rec.filter((f, i) => i && f.clips.find(c => c.name === clip) && rec[i - 1].clips.find(c => c.name === clip) && f.clips.find(c => c.name === clip).time + 0.05 < rec[i - 1].clips.find(c => c.name === clip).time).length,
      smooth: smooth(rec), extra: ex, startedFromYawDeg: r1(D(s0.move.yaw)), screenshots: shots,
    };
    const st1 = await st(); res.lastOneShot = st1.anim.lastOneShot;
    console.log('ONESHOT', key, JSON.stringify(res)); return res;
  }
  O.space_Jump = await oneShot('Space', 'Jump', async shots => {
    await sleep(1300); const apex = await st(); shots.push(await shot('c11_jump_space'));
    // Conflicts while Jump plays: 3 (Squat) and Space again are rejected; W is ignored.
    await page.keyboard.press('Digit3'); const rej = (await st()).anim.lastOneShot;
    await page.keyboard.down('KeyW'); await sleep(1200); const w = await st(); await page.keyboard.up('KeyW');
    return { apexPelvisY: r3(apex.pelvisY), apexOneShot: apex.anim.oneShot, squatDuringJump: rej, wDuringJump: { speed: w.move.speed, pos: w.move.position, oneShot: w.anim.oneShot && w.anim.oneShot.name } };
  });
  O.digit1_ToPlank = await oneShot('Digit1', 'ToPlank', async shots => { await waitSim(4.4); shots.push(await shot('c12_toplank_1_hold')); const s = await st(); return { atHold: s.anim.oneShot }; });
  O.digit2_ToBridge = await oneShot('Digit2', 'ToBridge', async shots => { await waitSim(4.6); shots.push(await shot('c13_tobridge_2_hold')); const s = await st(); return { atHold: s.anim.oneShot }; });
  O.digit3_Squat = await oneShot('Digit3', 'Squat', async shots => { await waitSim(1.3); shots.push(await shot('c14_squat_3')); return null; });
  O.numpad1_ToPlank_releasedByW = await oneShot('Numpad1', 'ToPlank', async () => {
    await waitSim(4.3); const h = await st(); await page.keyboard.down('KeyW'); await sleep(4000); const w = await st(); await page.keyboard.up('KeyW');
    return { holdBeforeW: h.anim.oneShot, afterW: { loco: w.locoState, speed: r3(w.move.speed), oneShot: w.anim.oneShot } };
  });
  // Debugger preview still plays one-shots and Walk_Back (manual mode); keys rejected there.
  await waitIdle();
  await page.click('#anim-debug [data-action="mode"]'); await blur();
  const prev = {};
  for (const clip of ['Jump', 'Walk_Back']) {
    await page.select('#anim-debug select', clip); await page.click('#anim-debug [data-action="preview"]'); await blur();
    await waitSim(1.0); const s = await st(); prev[clip] = { state: s.anim.state, previewClip: s.anim.previewClip, weight: r3(s.anim.clips.find(c => c.name === clip).weight), time: r3(s.anim.clips.find(c => c.name === clip).time) };
    if (clip === 'Walk_Back') await shot('c15_debugger_preview_walkback');
  }
  await page.keyboard.press('Space'); prev.spaceInManual = (await st()).anim.lastOneShot;
  await page.click('#anim-debug [data-action="mode"]'); await blur(); prev.backToAuto = (await st()).anim.mode; prev.idle = await waitIdle();
  O.debuggerPreview = prev;
  const ro = await page.$$eval('#anim-debug .ad-live dt', d => d.map(x => x.textContent));
  O.debuggerFields = ro;
  O.debuggerReadout = await page.$$eval('#anim-debug .ad-live dd', d => Object.fromEntries(d.map(x => [x.dataset.field, x.textContent])));
  evidence.checks.oneShots = O;
}

// ===== 8b. Jump while walking =====
if (want('walkjump')) {
  // Jump while walking: the root brakes (no added displacement), then locomotion resumes on input.
  await setup(0, 'BACK');
  await waitIdle();
  const { rec: wj } = await record(async () => { await page.keyboard.down('KeyW'); await sleep(3500); await page.keyboard.press('Space'); await waitSim(5.0); await page.keyboard.up('KeyW'); await sleep(2500); });
  save('walk_then_jump', wj);
  const tj = wj.findIndex(f => f.oneShot); const v0 = wj[tj - 1]?.speed ?? 0;
  const stopI = wj.findIndex((f, i) => i > tj && f.speed === 0);
  const resumeI = wj.findIndex((f, i) => i > tj && !f.oneShot && f.speed > 0.05);
  evidence.checks.walkThenJump = { speedAtTrigger: r3(v0), brakeDistM: stopI > 0 ? r3(Math.hypot(wj[stopI].x - wj[tj - 1].x, wj[stopI].z - wj[tj - 1].z)) : null, theoreticalBrakeDistM: r3(v0 * v0 / 16), stoppedAfterSimS: stopI > 0 ? r3(wj[stopI].t - wj[tj].t) : null,
    movedDuringJumpAfterStopM: stopI > 0 ? r3(Math.max(...wj.filter((f, i) => i >= stopI && f.oneShot).map(f => Math.hypot(f.x - wj[stopI].x, f.z - wj[stopI].z)))) : null,
    resumedWalkingAfterJump: resumeI > 0, resumeAtSimS: resumeI > 0 ? r3(wj[resumeI].t - wj[tj].t) : null, smooth: smooth(wj) };
  console.log('WALK+JUMP', JSON.stringify(evidence.checks.walkThenJump));
  await waitIdle();
}

// ===== 9. Overview screenshot with the debugger readouts while moving =====
if (want('overview')) {
  await setup(0.3, 'RESET');
  await page.keyboard.down('KeyA'); await sleep(4500);
  evidence.checks.overview = { screenshot: await shot('c16_moving_debugger_readouts'), readout: await page.$$eval('#anim-debug .ad-live dd', d => Object.fromEntries(d.map(x => [x.dataset.field, x.textContent]))) };
  await page.keyboard.up('KeyA'); await waitIdle();
}

evidence.errors = errors;
evidence.oneShotLog = logs.filter(l => l.includes('[animation] one-shot'));
writeFileSync(`${OUT}/validation_evidence${ONLY ? '_' + [...ONLY].join('_') : ''}.json`, JSON.stringify(evidence, null, 1));
console.log('FPS', evidence.fps, 'LOAD', evidence.loadMs, 'ERRORS', JSON.stringify(errors));
await browser.close();
