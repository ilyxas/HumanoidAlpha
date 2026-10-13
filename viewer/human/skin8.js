// 8-bone skinning for three.js SkinnedMesh.
//
// three.js skins with 4 influences per vertex (skinIndex/skinWeight). Xandra's
// body uses up to 12 (corrective and twist bones), so 4-bone skinning tears
// tight clothing in deep poses (Squat). The GLB is exported with 8 influences:
// JOINTS_0/WEIGHTS_0 + JOINTS_1/WEIGHTS_1; GLTFLoader keeps the second set as
// `joints_1`/`weights_1`. This module adds that set to the skinning shader.
//
// GLTFLoader also calls normalizeSkinWeights(), which rescales the first four
// weights to sum 1. The exporter normalizes all eight to 1, so the first four
// are restored as (1 - sum(weights_1)) of their normalized values.
// Meshes without joints_1 (e.g. older GLBs) are left untouched.
import * as THREE from 'three';

const PARS = `#include <skinning_pars_vertex>
#ifdef USE_SKINNING
	attribute vec4 joints_1;
	attribute vec4 weights_1;
#endif`;

const BASE = `#include <skinbase_vertex>
#ifdef USE_SKINNING
	mat4 boneMatX1 = getBoneMatrix( joints_1.x );
	mat4 boneMatY1 = getBoneMatrix( joints_1.y );
	mat4 boneMatZ1 = getBoneMatrix( joints_1.z );
	mat4 boneMatW1 = getBoneMatrix( joints_1.w );
#endif`;

const NORMAL = `#ifdef USE_SKINNING
	mat4 skinMatrix = mat4( 0.0 );
	skinMatrix += skinWeight.x * boneMatX;
	skinMatrix += skinWeight.y * boneMatY;
	skinMatrix += skinWeight.z * boneMatZ;
	skinMatrix += skinWeight.w * boneMatW;
	skinMatrix += weights_1.x * boneMatX1;
	skinMatrix += weights_1.y * boneMatY1;
	skinMatrix += weights_1.z * boneMatZ1;
	skinMatrix += weights_1.w * boneMatW1;
	skinMatrix = bindMatrixInverse * skinMatrix * bindMatrix;
	objectNormal = vec4( skinMatrix * vec4( objectNormal, 0.0 ) ).xyz;
	#ifdef USE_TANGENT
		objectTangent = vec4( skinMatrix * vec4( objectTangent, 0.0 ) ).xyz;
	#endif
#endif`;

const POSITION = `#ifdef USE_SKINNING
	vec4 skinVertex = bindMatrix * vec4( transformed, 1.0 );
	vec4 skinned = vec4( 0.0 );
	skinned += boneMatX * skinVertex * skinWeight.x;
	skinned += boneMatY * skinVertex * skinWeight.y;
	skinned += boneMatZ * skinVertex * skinWeight.z;
	skinned += boneMatW * skinVertex * skinWeight.w;
	skinned += boneMatX1 * skinVertex * weights_1.x;
	skinned += boneMatY1 * skinVertex * weights_1.y;
	skinned += boneMatZ1 * skinVertex * weights_1.z;
	skinned += boneMatW1 * skinVertex * weights_1.w;
	transformed = ( bindMatrixInverse * skinned ).xyz;
#endif`;

function patchShader(shader) {
  shader.vertexShader = shader.vertexShader
    .replace('#include <skinning_pars_vertex>', PARS)
    .replace('#include <skinbase_vertex>', BASE)
    .replace('#include <skinnormal_vertex>', NORMAL)
    .replace('#include <skinning_vertex>', POSITION);
}

function eightBoneMaterial(material, cache) {
  if (cache.has(material)) return cache.get(material);
  const m = material.clone();
  m.onBeforeCompile = patchShader;
  m.customProgramCacheKey = () => 'skin8';
  cache.set(material, m);
  return m;
}

function restoreFirstWeights(geometry) {
  const w0 = geometry.getAttribute('skinWeight'), w1 = geometry.getAttribute('weights_1');
  for (let i = 0; i < w0.count; i++) {
    const s0 = w0.getX(i) + w0.getY(i) + w0.getZ(i) + w0.getW(i);
    const s1 = w1.getX(i) + w1.getY(i) + w1.getZ(i) + w1.getW(i);
    if (s0 <= 0 || s1 <= 0) continue;
    const k = Math.max(0, 1 - s1) / s0;
    w0.setXYZW(i, w0.getX(i) * k, w0.getY(i) * k, w0.getZ(i) * k, w0.getW(i) * k);
  }
  w0.needsUpdate = true;
}

/** Enable 8-bone skinning on every SkinnedMesh under `root` that carries joints_1/weights_1. Returns the mesh count. */
export function enableEightBoneSkinning(root) {
  const cache = new Map();
  let count = 0;
  root.traverse(o => {
    if (!o.isSkinnedMesh) return;
    const g = o.geometry;
    if (!g.getAttribute('joints_1') || !g.getAttribute('weights_1')) return;
    restoreFirstWeights(g);
    o.material = Array.isArray(o.material) ? o.material.map(m => eightBoneMaterial(m, cache)) : eightBoneMaterial(o.material, cache);
    const mats = Array.isArray(o.material) ? o.material : [o.material];
    if (mats.every(m => !m.transparent && !(m.alphaTest > 0))) {   // opaque: shadow pass with the same skinning
      const depth = new THREE.MeshDepthMaterial({ depthPacking: THREE.RGBADepthPacking });
      depth.onBeforeCompile = patchShader; depth.customProgramCacheKey = () => 'skin8-depth';
      o.customDepthMaterial = depth;
    }
    count++;
  });
  return count;
}
