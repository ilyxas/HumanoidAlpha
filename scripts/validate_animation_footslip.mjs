// EXP002.1-B foot-skate check at playback x0.25 (sim dt ~0.025 s/frame). Needs ./START.command animation running.
import puppeteer from 'puppeteer-core';
import { writeFileSync } from 'node:fs';
const OUT = process.env.OUT || new URL('../reports/exp002_1b', import.meta.url).pathname;
const browser = await puppeteer.launch({ headless: 'new', executablePath: process.env.CHROME || '/usr/bin/google-chrome',
  args: ['--no-sandbox', '--enable-unsafe-swiftshader', '--use-gl=angle', '--use-angle=swiftshader-webgl', '--ignore-gpu-blocklist', '--window-size=1280,800'],
  defaultViewport: { width: 1280, height: 800 } });
const page = await browser.newPage();
const errors = [];
page.on('pageerror', e => errors.push(e.message));
await page.goto('http://127.0.0.1:8788/', { waitUntil: 'domcontentloaded' });
await page.waitForFunction(() => window.__animation?.ready, { timeout: 240000 });
const sleep = ms => new Promise(r => setTimeout(r, ms));
await sleep(1000);
await page.click('button[data-view="FRONT"]');
await page.$eval('#anim-debug input[data-action="speed"]', el => { el.value = '0.25'; el.dispatchEvent(new Event('input', { bubbles: true })); });
await page.evaluate(() => document.activeElement?.blur());
const NAT = { Walk_Fwd: 1.442, Run_Fwd: 3.016, Walk_Back: 0.647 };
function slip(rec, label) {
  const bones = ['ball_l', 'ball_r', 'foot_l', 'foot_r'];
  const res = {};
  for (const b of bones) {
    const ys = rec.map(f => f.feet[b][1]); const minY = Math.min(...ys);
    const planted = []; const swingRel = [];
    for (let i = 1; i < rec.length; i++) {
      const a = rec[i - 1], f = rec[i];
      if (!(f.clips.length === 1 && f.clips[0].w > 0.999 && NAT[f.clips[0].name])) continue;
      const v = Math.hypot(f.feet[b][0] - a.feet[b][0], f.feet[b][2] - a.feet[b][2]) / f.dt;
      if (a.feet[b][1] < minY + 0.015 && f.feet[b][1] < minY + 0.015) planted.push(v);
    }
    planted.sort((x, y) => x - y);
    res[b] = { plantedSamples: planted.length, median: +(planted[Math.floor(planted.length / 2)] ?? NaN).toFixed(3), p90: +(planted[Math.floor(planted.length * 0.9)] ?? NaN).toFixed(3) };
  }
  const body = rec.filter(f => f.clips.length === 1 && NAT[f.clips[0].name]).map(f => Math.abs(f.speed));
  return { label, frames: rec.length, meanDt: +(rec.reduce((s, f) => s + f.dt, 0) / rec.length).toFixed(4), bodySpeed: +(body.reduce((s, v) => s + v, 0) / body.length).toFixed(3), clip: rec.at(-1).clips.map(c => c.name).join('+'), bones: res };
}
const results = [];
async function run(label, keys, settleMs, recMs) {
  for (const k of keys) await page.keyboard.down(k);
  await sleep(settleMs);
  await page.evaluate(() => window.__animation.record(true));
  await sleep(recMs);
  const rec = await page.evaluate(() => window.__animation.takeRecording());
  for (const k of [...keys].reverse()) await page.keyboard.up(k);
  const r = slip(rec, label); results.push(r); console.log(JSON.stringify(r));
  if (process.env.RAW) writeFileSync(`${process.env.RAW}/rec_hires_${label}.json`, JSON.stringify(rec));
  await page.waitForFunction(() => window.__animation.state().locoState === 'Idle' && window.__animation.state().move.speed === 0, { timeout: 120000 });
  await sleep(1500);
}
await run('walk_fwd', ['KeyW'], 12000, 30000);
await run('run_fwd', ['KeyW', 'Shift'], 16000, 30000);
await run('walk_back', ['KeyS'], 16000, 40000);
writeFileSync(`${OUT}/footslip_hires.json`, JSON.stringify({ results, errors }, null, 1));
console.log('ERRORS', errors);
await browser.close();
