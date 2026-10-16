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

/**
 * Load a report and fill in clips it lacks from the other reports of the same folder (index.json,
 * written by physics/clip_collisions.py / clip_react.py). A GLB renamed to Xandra_Animated.glb then
 * still finds its clips. Own clips win; a missing own report is fine if the index has the clip.
 */
export async function loadReport(url, fetchJson = u => fetch(u, { cache: 'no-cache' }).then(r => (r.ok ? r.json() : null))) {
  const own = await fetchJson(url).catch(() => null);
  const dir = url.slice(0, url.lastIndexOf('/') + 1), self = url.slice(dir.length);
  const index = await fetchJson(dir + 'index.json').catch(() => null);
  let merged = own ? { ...own, clips: { ...own.clips }, sources: { } } : null;
  if (merged) for (const n of Object.keys(own.clips)) merged.sources[n] = own.glb;
  for (const r of index?.reports || []) {
    if (r.file === self || !r.clips.some(n => !merged?.clips[n])) continue;
    const other = await fetchJson(dir + r.file).catch(() => null);
    if (!other) continue;
    if (!merged) merged = { ...other, clips: {}, sources: {} };
    for (const [n, c] of Object.entries(other.clips)) if (!merged.clips[n]) { merged.clips[n] = c; merged.sources[n] = other.glb; }
  }
  if (!merged) throw new Error(`no report at ${url}`);
  return merged;
}
