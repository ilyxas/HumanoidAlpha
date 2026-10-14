/**
 * Eight-bone skinning for three.js r170.
 *
 * Stock three.js skins with four influences (JOINTS_0/WEIGHTS_0). GLTFLoader keeps
 * JOINTS_1/WEIGHTS_1 as plain attributes `joints_1`/`weights_1` and then calls
 * normalizeSkinWeights(), which rescales the first four weights to sum to 1.
 * Xandra's GLB is exported with eight influences (garment edges and lace tear
 * with four), so for every skinned mesh that carries a second joint set this:
 *   - undoes the loader's renormalisation (first four scaled back by 1 - sum(weights_1));
 *   - patches the material to add four more bones (shadows keep the stock four-bone depth pass).
 * Meshes without `joints_1` are left untouched. Call once, right after loading.
 */
import * as THREE from 'three';

const EXTRA_BONES = `
#ifdef USE_SKINNING
  mat4 boneMat1X = getBoneMatrix( joints_1.x );
  mat4 boneMat1Y = getBoneMatrix( joints_1.y );
  mat4 boneMat1Z = getBoneMatrix( joints_1.z );
  mat4 boneMat1W = getBoneMatrix( joints_1.w );
#endif`;

function patchShader(shader) {
  // onBeforeCompile sees unexpanded #include lines: expand the skinning chunks before editing them.
  shader.vertexShader = 'attribute vec4 joints_1;\nattribute vec4 weights_1;\n' + shader.vertexShader
    .replace('#include <skinbase_vertex>', '#include <skinbase_vertex>' + EXTRA_BONES)
    .replace('#include <skinning_vertex>', THREE.ShaderChunk.skinning_vertex.replace(
      'skinned += boneMatW * skinVertex * skinWeight.w;',
      'skinned += boneMatW * skinVertex * skinWeight.w;\n\tskinned += boneMat1X * skinVertex * weights_1.x;\n\tskinned += boneMat1Y * skinVertex * weights_1.y;\n\tskinned += boneMat1Z * skinVertex * weights_1.z;\n\tskinned += boneMat1W * skinVertex * weights_1.w;'))
    .replace('#include <skinnormal_vertex>', THREE.ShaderChunk.skinnormal_vertex.replace(
      'skinMatrix += skinWeight.w * boneMatW;',
      'skinMatrix += skinWeight.w * boneMatW;\n\tskinMatrix += weights_1.x * boneMat1X;\n\tskinMatrix += weights_1.y * boneMat1Y;\n\tskinMatrix += weights_1.z * boneMat1Z;\n\tskinMatrix += weights_1.w * boneMat1W;'));
}

function eightBone(material) {
  const m = material.clone();
  m.onBeforeCompile = patchShader;
  m.customProgramCacheKey = () => 'skin8';
  return m;
}

/** Restore the first four weights so all eight sum to 1 again (undo normalizeSkinWeights). */
export function restoreWeights(skinWeight, weights1) {
  for (let i = 0; i < skinWeight.count; i++) {
    const rest = 1 - (weights1.getX(i) + weights1.getY(i) + weights1.getZ(i) + weights1.getW(i));
    skinWeight.setXYZW(i, skinWeight.getX(i) * rest, skinWeight.getY(i) * rest, skinWeight.getZ(i) * rest, skinWeight.getW(i) * rest);
  }
  skinWeight.needsUpdate = true;
}

export function enableEightBoneSkinning(root) {
  const restored = new WeakSet();
  let patched = 0, skinned = 0;
  root.traverse(o => {
    if (!o.isSkinnedMesh) return;
    skinned++;
    const g = o.geometry, j1 = g.attributes.joints_1, w1 = g.attributes.weights_1;
    if (!j1 || !w1) return;
    if (!restored.has(g)) { restored.add(g); restoreWeights(g.attributes.skinWeight, w1); }
    o.material = Array.isArray(o.material) ? o.material.map(eightBone) : eightBone(o.material);
    o.frustumCulled = false; // bounds come from four-bone CPU skinning
    patched++;
  });
  return { patched, skinned };
}
