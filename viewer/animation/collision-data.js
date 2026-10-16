// Pure helpers for the COLLISIONS overlay (no three.js, so node tests can import them).
// Report format: physics/clip_collisions.py ("xandra-clip-collisions/1").

/** Report point (Blender/MuJoCo: Z up, metres) -> GLB scene local [x, y, z] (Y up). */
export function glbFromReport([x, y, z]) { return [x, z, -y]; }

/** Frame of a clip at time t (loop wraps, one-shots clamp). */
export function frameAt(clip, time) {
  const n = clip.frames.length;
  if (!n) return null;
  let t = time;
  if (clip.loop !== false && clip.duration > 0 && t > clip.duration) t %= clip.duration;
  return clip.frames[Math.max(0, Math.min(n - 1, Math.round(t * clip.fps)))];
}

/** Dominant clip of an animation snapshot: { name, time, weight } or null. */
export function dominantClip(snapshot) {
  let best = null;
  for (const c of snapshot?.clips || []) if (c.weight > 0.001 && (!best || c.weight > best.weight)) best = c;
  return best && { name: best.name, time: best.time, weight: best.weight };
}
