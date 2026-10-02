import puppeteer from 'puppeteer-core';
import fs from 'fs';
const browser = await puppeteer.launch({
  headless: 'new', executablePath: '/usr/bin/google-chrome-stable',
  args: ['--no-sandbox','--disable-gpu','--enable-unsafe-swiftshader','--use-gl=angle','--use-angle=swiftshader-webgl'],
  defaultViewport: { width: 800, height: 600 },
});
const page = await browser.newPage();
await page.goto('http://127.0.0.1:8787/viewer/index.html', { waitUntil: 'domcontentloaded' });
await page.waitForFunction(() => window.__HA?.ready(), { timeout: 120000 });

// Hook: after physics apply, compute errors
const result = await page.evaluate(async () => {
  // Monkey patch to capture last pose
  let last = null;
  const orig = window.__HA;
  // connect physics
  document.getElementById('btnPhysics').click();
  await new Promise(r => setTimeout(r, 2000));

  // Access internals via scene
  const THREE = await import('./vendor/three/three.module.js');
  // We need bones and last pose - add temporary globals by re-applying
  // Read stats
  const stats = window.__HA.getStats();

  // Instrument: fetch one pose via WS ourselves
  const pose = await new Promise((resolve, reject) => {
    const ws = new WebSocket('ws://127.0.0.1:8765');
    ws.onmessage = (ev) => {
      const msg = JSON.parse(ev.data);
      if (msg.type === 'pose') { resolve(msg.pose); ws.close(); }
    };
    ws.onerror = reject;
    setTimeout(() => reject(new Error('timeout')), 5000);
  });

  // Find bones in scene
  const bones = new Map();
  // traverse from window - need scene access. Expose via __HA
  return { stats, nBonesPose: Object.keys(pose.bone_world_quat).length, sample: Object.keys(pose.bone_world_quat).slice(0,5) };
});
console.log(JSON.stringify(result, null, 2));

// Better: inject diagnostic into page with exposed bones
const err = await page.evaluate(async () => {
  const THREE = await import('/viewer/vendor/three/three.module.js');
  // Get pose
  const pose = await new Promise((resolve, reject) => {
    const ws = new WebSocket('ws://127.0.0.1:8765');
    ws.onmessage = (ev) => {
      const msg = JSON.parse(ev.data);
      if (msg.type === 'pose') { resolve(msg.pose); ws.close(); }
    };
    ws.onerror = () => reject(new Error('ws'));
    setTimeout(() => reject(new Error('timeout')), 5000);
  });

  // Apply then measure using same math as main.js
  // We need bone refs - expose them
  if (!window.__HA._bones) {
    // can't access closed-over bonesByName - dump from SkeletonHelper
  }
  return { apply_order: pose.apply_order, root: pose.root_pos };
});
console.log('err', err);
await browser.close();
