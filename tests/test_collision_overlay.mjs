// COLLISIONS overlay helpers + the committed report. Run: node tests/test_collision_overlay.mjs
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { dominantClip, frameAt, glbFromReport } from '../viewer/animation/collision-data.js';

// Blender/MuJoCo (Z up, she faces -Y) -> GLB (Y up, she faces +Z); inverse of clip_retarget's root mapping.
assert.deepEqual(glbFromReport([1, 2, 3]), [1, 3, -2]);

const clip = { duration: 1, fps: 10, loop: true, frames: Array.from({ length: 11 }, (_, i) => ({ t: i / 10, hits: [] })) };
assert.equal(frameAt(clip, 0.34).t, 0.3);
assert.equal(frameAt(clip, 1.26).t, 0.3); // loop wraps
assert.equal(frameAt({ ...clip, loop: false }, 5).t, 1); // one-shot clamps

assert.equal(dominantClip({ clips: [] }), null);
assert.deepEqual(dominantClip({ clips: [{ name: 'Idle', weight: 0.3, time: 1 }, { name: 'Jump', weight: 0.7, time: 0.5 }] }),
  { name: 'Jump', time: 0.5, weight: 0.7 });

const report = JSON.parse(readFileSync(new URL('../reports/clip_collisions/Xandra_Animated.collisions.json', import.meta.url)));
assert.equal(report.format, 'xandra-clip-collisions/1');
for (const [name, c] of Object.entries(report.clips)) {
  assert.ok(c.frames.length > 1 && c.frames.at(-1).t <= c.duration + 1e-6 && c.duration - c.frames.at(-1).t <= 1 / c.fps + 1e-3, name);
  for (const f of c.frames) for (const h of f.hits) {
    assert.ok(['self', 'joint', 'floor'].includes(h.kind));
    assert.equal(h.pa.length, 3); assert.equal(h.pb.length, 3);
    const d = Math.hypot(h.pa[0] - h.pb[0], h.pa[1] - h.pb[1], h.pa[2] - h.pb[2]) * 1000;
    assert.ok(Math.abs(d - h.depth_mm) < 1.0, `${name} ${f.t} ${h.a}/${h.b}: |pa-pb| ${d} vs ${h.depth_mm}`);
  }
}
console.log('test_collision_overlay OK');
