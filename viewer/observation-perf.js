// Opt-in developer diagnostics. No model state is retained or displayed.
export function createProfiler(enabled) {
  let start=performance.now(), lastFrame=null, matrixDepth=0;
  let samples={}, poses=0, frames=0, longFrames=0, estimatedMissed60Hz=0;
  function sample(key,ms){if(enabled)(samples[key]??=[]).push(ms);}
  function frame(t){
    if(!enabled)return;
    frames++;
    if(lastFrame!==null){const gap=t-lastFrame;sample('frameIntervalMs',gap);if(gap>50)longFrames++;estimatedMissed60Hz+=Math.max(0,Math.round(gap/(1000/60))-1);}
    lastFrame=t;
    if(t-start>=5000){
      const seconds=(t-start)/1000, timing={};
      for(const [key,a] of Object.entries(samples)){a.sort((a,b)=>a-b);timing[key]={count:a.length,mean:a.reduce((x,y)=>x+y,0)/a.length,p95:a[Math.floor((a.length-1)*.95)],max:a.at(-1)};}
      console.info('[observation-perf]',JSON.stringify({wallTime:new Date().toISOString(),windowSeconds:seconds,framesPerSecond:frames/seconds,poseMessagesPerSecond:poses/seconds,longFramesOver50ms:longFrames,estimatedMissed60Hz,timing}));
      start=t;samples={};poses=frames=longFrames=estimatedMissed60Hz=0;
    }
  }
  function instrument(root){
    if(!enabled)return;
    const skeletons=new Set();
    root.traverse(o=>{
      for(const name of ['updateMatrixWorld','updateWorldMatrix']){
        const original=o[name];
        o[name]=function(...args){const outer=matrixDepth++===0,t=outer?performance.now():0;try{return original.apply(this,args);}finally{matrixDepth--;if(outer)sample('modelWorldMatricesCpuMs',performance.now()-t);}};
      }
      if(o.isSkinnedMesh)skeletons.add(o.skeleton);
    });
    for(const skeleton of skeletons){const update=skeleton.update;skeleton.update=function(...args){const t=performance.now();try{return update.apply(this,args);}finally{sample('skinPaletteCpuMs',performance.now()-t);}};}
  }
  return {enabled,sample,frame,instrument,pose(){if(enabled)poses++;},event(name,details={}){if(enabled)console.info('[observation-event]',JSON.stringify({wallTime:new Date().toISOString(),name,...details}));}};
}
