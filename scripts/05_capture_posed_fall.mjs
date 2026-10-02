import puppeteer from 'puppeteer-core';
import fs from 'fs';
import path from 'path';
import { fileURLToPath } from 'url';
import { spawn } from 'child_process';
const __dirname = path.dirname(fileURLToPath(import.meta.url));
const ROOT = path.resolve(__dirname, '..');
const OUT = path.join(ROOT, 'shots');
const PY = process.env.PY || (process.env.VIRTUAL_ENV ? process.env.VIRTUAL_ENV + '/bin/python' : 'python3');

function sleep(ms){ return new Promise(r=>setTimeout(r,ms)); }

async function withBridge(scenario, fn) {
  // kill existing on 8765
  try {
    const { execSync } = await import('child_process');
    execSync("fuser -k 8765/tcp 2>/dev/null || true");
  } catch(_){}
  await sleep(400);
  const log = fs.openSync(`/tmp/ha_ws_${scenario}.log`, 'w');
  const child = spawn(PY, ['physics/ws_bridge.py', '--scenario', scenario, '--broadcast-hz', '30'], {
    cwd: ROOT, stdio: ['ignore', log, log],
  });
  await sleep(1500);
  try {
    await fn();
  } finally {
    child.kill('SIGTERM');
    await sleep(300);
  }
}

async function launchPage() {
  const browser = await puppeteer.launch({
    headless: 'new',
    executablePath: '/usr/bin/google-chrome-stable',
    args: ['--no-sandbox','--disable-gpu','--enable-unsafe-swiftshader','--use-gl=angle','--use-angle=swiftshader-webgl','--window-size=1280,800'],
    defaultViewport: { width: 1280, height: 800 },
  });
  const page = await browser.newPage();
  page.on('pageerror', e => console.log('PAGEERROR', e.message));
  await page.goto('http://127.0.0.1:8787/viewer/index.html', { waitUntil: 'domcontentloaded', timeout: 60000 });
  await page.waitForFunction(() => window.__HA && window.__HA.ready(), { timeout: 180000 });
  await sleep(800);
  return { browser, page };
}

await withBridge('posed', async () => {
  const { browser, page } = await launchPage();
  await page.evaluate(() => window.__HA.modePhysics());
  await page.waitForFunction(() => window.__HA.getStats().physicsMsgHz > 0 || window.__HA.getStats().lastPoseTime !== 0, { timeout: 20000 });
  await sleep(1000);
  await page.screenshot({ path: path.join(OUT, '20_posed_knee.png'), type: 'png' });
  const st = await page.evaluate(() => window.__HA.getStats());
  fs.writeFileSync(path.join(OUT, 'posed_stats.json'), JSON.stringify(st, null, 2));
  console.log('POSED', st);
  await browser.close();
});

await withBridge('fall', async () => {
  const { browser, page } = await launchPage();
  await page.evaluate(() => window.__HA.modePhysics());
  await page.waitForFunction(() => window.__HA.getStats().physicsMsgHz > 0, { timeout: 20000 });
  for (let i = 0; i < 8; i++) {
    await sleep(400);
    const name = `30_fall_${String(i).padStart(2,'0')}.png`;
    await page.screenshot({ path: path.join(OUT, name), type: 'png' });
    const st = await page.evaluate(() => window.__HA.getStats());
    console.log('FALL', name, 't', st.lastPoseTime, 'hz', st.physicsMsgHz, 'fps', st.fps, 'lat', st.latencyMs);
  }
  const st = await page.evaluate(() => window.__HA.getStats());
  fs.writeFileSync(path.join(OUT, 'fall_stats.json'), JSON.stringify(st, null, 2));
  await browser.close();
});

console.log('DONE');
