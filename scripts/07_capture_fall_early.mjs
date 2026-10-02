import puppeteer from 'puppeteer-core';
import fs from 'fs';
import path from 'path';
import { spawn } from 'child_process';
import { fileURLToPath } from 'url';
const ROOT = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const OUT = path.join(ROOT, 'shots');
const PY = process.env.PY || (process.env.VIRTUAL_ENV ? process.env.VIRTUAL_ENV + '/bin/python' : 'python3');
const sleep = (ms) => new Promise(r => setTimeout(r, ms));

// Start page FIRST so client is ready when bridge waits
const browser = await puppeteer.launch({
  headless: 'new', executablePath: '/usr/bin/google-chrome-stable',
  args: ['--no-sandbox','--disable-gpu','--enable-unsafe-swiftshader','--use-gl=angle','--use-angle=swiftshader-webgl','--window-size=1280,800'],
  defaultViewport: { width: 1280, height: 800 },
});
const page = await browser.newPage();
await page.goto('http://127.0.0.1:8787/viewer/index.html', { waitUntil: 'domcontentloaded' });
await page.waitForFunction(() => window.__HA?.ready(), { timeout: 120000 });

const log = fs.openSync('/tmp/ha_ws_fall2.log', 'w');
const child = spawn(PY, ['physics/ws_bridge.py', '--scenario', 'fall', '--broadcast-hz', '30'], { cwd: ROOT, stdio: ['ignore', log, log] });
await sleep(800);
await page.evaluate(() => window.__HA.modePhysics());
await page.waitForFunction(() => window.__HA.getStats().physicsMsgHz > 0, { timeout: 20000 });

for (let i = 0; i < 10; i++) {
  await sleep(250);
  const st = await page.evaluate(() => window.__HA.getStats());
  const name = `31_fall_early_${String(i).padStart(2,'0')}.png`;
  await page.screenshot({ path: path.join(OUT, name), type: 'png' });
  console.log(name, 't', st.lastPoseTime, 'fps', st.fps, 'hz', st.physicsMsgHz, 'lat', st.latencyMs);
}
const st = await page.evaluate(() => window.__HA.getStats());
fs.writeFileSync(path.join(OUT, 'fall_early_stats.json'), JSON.stringify(st, null, 2));
child.kill();
await browser.close();
