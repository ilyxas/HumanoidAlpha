import puppeteer from 'puppeteer-core';
import fs from 'fs';
import path from 'path';
import { fileURLToPath } from 'url';
const __dirname = path.dirname(fileURLToPath(import.meta.url));
const ROOT = path.resolve(__dirname, '..');
const OUT = path.join(ROOT, 'shots');
fs.mkdirSync(OUT, { recursive: true });

const browser = await puppeteer.launch({
  headless: 'new',
  executablePath: '/usr/bin/google-chrome-stable',
  args: [
    '--no-sandbox', '--disable-gpu', '--enable-unsafe-swiftshader',
    '--use-gl=angle', '--use-angle=swiftshader-webgl', '--window-size=1280,800',
  ],
  defaultViewport: { width: 1280, height: 800 },
});
const page = await browser.newPage();
page.on('console', (msg) => {
  const t = msg.text();
  if (t.includes('PAGEERROR') || t.includes('error') || t.includes('WS')) console.log('CONSOLE', t.slice(0, 180));
});
page.on('pageerror', (e) => console.log('PAGEERROR', e.message));

await page.goto('http://127.0.0.1:8787/viewer/index.html', { waitUntil: 'domcontentloaded', timeout: 60000 });
await page.waitForFunction(() => window.__HA && window.__HA.ready(), { timeout: 180000 });
await new Promise(r => setTimeout(r, 1000));

// re-shot shoulder with new axis
await page.evaluate(() => { window.__HA.reset(); window.__HA.setShoulder(1.2, 0.5); });
await new Promise(r => setTimeout(r, 500));
await page.screenshot({ path: path.join(OUT, '04b_shoulder_yaxis.png'), type: 'png' });
console.log('SHOT 04b');

await page.evaluate(() => window.__HA.modePhysics());
// wait for physics messages
await page.waitForFunction(() => {
  const s = window.__HA.getStats();
  return s.physicsMsgHz > 0 || s.lastPoseTime > 0;
}, { timeout: 30000 });
console.log('physics streaming');

for (let i = 0; i < 6; i++) {
  await new Promise(r => setTimeout(r, 800));
  const name = `10_physics_knee_${String(i).padStart(2,'0')}.png`;
  await page.screenshot({ path: path.join(OUT, name), type: 'png' });
  const stats = await page.evaluate(() => window.__HA.getStats());
  console.log('SHOT', name, 'sim', stats.lastPoseTime, 'hz', stats.physicsMsgHz, 'fps', stats.fps, 'lat', stats.latencyMs);
}

const finalStats = await page.evaluate(() => window.__HA.getStats());
fs.writeFileSync(path.join(OUT, 'physics_knee_stats.json'), JSON.stringify(finalStats, null, 2));
console.log('FINAL', JSON.stringify(finalStats));
await browser.close();
