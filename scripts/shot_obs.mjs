import puppeteer from 'puppeteer-core';
const out = process.argv[2] || '/workspace/arm_raise_v2/shots/shot.png';
const view = (process.argv[3] || 'LEFT').toUpperCase();
const browser = await puppeteer.launch({
  headless: 'new',
  executablePath: '/usr/bin/google-chrome-stable',
  args: ['--no-sandbox','--disable-gpu','--enable-unsafe-swiftshader','--use-gl=angle','--use-angle=swiftshader-webgl','--window-size=1400,900'],
  defaultViewport: { width: 1400, height: 900 },
});
const page = await browser.newPage();
page.on('pageerror', e => console.log('PAGEERROR', e.message));
await page.goto('http://127.0.0.1:8788/viewer/observation.html?wsPort=8766', { waitUntil: 'domcontentloaded', timeout: 60000 });
// wait until scene has a root object with bones (model loaded) and a few frames rendered
await page.waitForFunction(() => {
  // heuristic: canvas non-black-ish after model+pose
  const c = document.querySelector('canvas');
  return !!c && c.width > 100;
}, { timeout: 60000 });
await new Promise(r => setTimeout(r, 4000)); // allow GLB + WS pose
await page.click(`button[data-view="${view}"]`).catch(()=>{});
await new Promise(r => setTimeout(r, 800));
await page.screenshot({ path: out, type: 'png' });
console.log('WROTE', out);
await browser.close();
