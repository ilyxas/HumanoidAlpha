import assert from 'node:assert/strict';
import { readFile } from 'node:fs/promises';
// Load the standalone ES module without changing the project's package settings.
const source=await readFile(new URL('../viewer/observation-perf.js',import.meta.url),'utf8');
const {createProfiler}=await import('data:text/javascript;base64,'+Buffer.from(source).toString('base64'));
const originalPerformance=globalThis.performance, originalLog=console.info;
let now=0,logs=[];
try{
  globalThis.performance={now:()=>now};console.info=(tag,json)=>logs.push([tag,JSON.parse(json)]);
  const p=createProfiler(true);
  p.sample('poseApplyCpuMs',2);p.sample('poseApplyCpuMs',4);
  p.pose();p.pose();p.frame(0);p.frame(16);p.frame(100);p.frame(6000);
  const r=logs.find(([tag])=>tag==='[observation-perf]')[1];
  assert.equal(r.windowSeconds,6);assert.equal(r.poseMessagesPerSecond,2/6);
  assert.equal(r.framesPerSecond,4/6);assert.equal(r.longFramesOver50ms,2);
  assert.equal(r.timing.poseApplyCpuMs.mean,3);assert.equal(r.timing.poseApplyCpuMs.max,4);
  p.event('model-load-complete',{elapsedMs:123});assert.equal(logs.at(-1)[1].elapsedMs,123);
  // Nested world-matrix traversal must not count a child twice.
  logs=[];now=0;
  const nested=createProfiler(true);
  const child={updateMatrixWorld(){now+=2;return 7;},updateWorldMatrix(){now+=2;}};
  const root={updateMatrixWorld(){now+=1;const value=child.updateMatrixWorld();now+=1;return value;},updateWorldMatrix(){},traverse(fn){fn(this);fn(child);}};
  nested.instrument(root);assert.equal(root.updateMatrixWorld(),7);nested.frame(6000);
  const sample=logs[0][1].timing.modelWorldMatricesCpuMs;
  assert.equal(sample.count,1);assert.equal(sample.mean,4);
  const disabled=createProfiler(false),before=root.updateMatrixWorld;
  disabled.instrument(root);assert.equal(root.updateMatrixWorld,before);
}finally{globalThis.performance=originalPerformance;console.info=originalLog;}
console.log('Observation profiler: rates use actual elapsed time; nested timings and disabled mode verified.');
