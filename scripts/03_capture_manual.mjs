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
    '--no-sandbox',
    '--disable-gpu',
    '--enable-unsafe-swiftshader',
    '--use-gl=angle',
    '--use-angle=swiftshader-webgl',
    '--window-size=1280,800',
  ],
  defaultViewport: { width: 1280, height: 800 },
});
const page = await browser.newPage();
page.on('console', (msg) => console.log('CONSOLE', msg.type(), msg.text().slice(0, 200)));
page.on('pageerror', (e) => console.log('PAGEERROR', e.message));
page.on('requestfailed', (req) => console.log('REQFAIL', req.url(), req.failure()?.errorText));

console.log('goto…');
await page.goto('http://127.0.0.1:8787/viewer/index.html', { waitUntil: 'domcontentloaded', timeout: 60000 });
console.log('wait ready…');
await page.waitForFunction(() => window.__HA && window.__HA.ready(), { timeout: 180000 });
console.log('ready');
await new Promise(r => setTimeout(r, 2500));

async function shot(name, fn) {
  if (fn) await fn();
  await new Promise(r => setTimeout(r, 600));
  const p = path.join(OUT, name);
  await page.screenshot({ path: p, type: 'png' });
  console.log('SHOT', name, fs.statSync(p).size);
}

await shot('01_rest.png', async () => { await page.evaluate(() => window.__HA.reset()); });
await shot('02_knee_neg1.png', async () => { await page.evaluate(() => window.__HA.setKnee(-1.0)); });
await shot('03_knee_neg2.png', async () => { await page.evaluate(() => window.__HA.setKnee(-2.0)); });
await shot('04_shoulder.png', async () => {
  await page.evaluate(() => { window.__HA.reset(); window.__HA.setShoulder(1.0, 0.4); });
});
await shot('05_knee_shoulder.png', async () => {
  await page.evaluate(() => { window.__HA.setKnee(-1.2); window.__HA.setShoulder(0.8, -0.3); });
});

const stats = await page.evaluate(() => window.__HA.getStats());
fs.writeFileSync(path.join(OUT, 'manual_stats.json'), JSON.stringify(stats, null, 2));
console.log('STATS', JSON.stringify(stats));
await browser.close();
