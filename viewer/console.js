const $ = id => document.getElementById(id);
let inventory=null,state=null,selected=null,kind='joint',connected=false,canControl=false,requestId=0;
let builtMode=null, fields=[], editingUntil=0,referenceQpos=null;
const referenceData=fetch('../reports/control_inventory.json').then(r=>{if(!r.ok)throw new Error('Reference state unavailable');return r.json();});
referenceData.catch(()=>{});
const regions=['Root','Torso','Head','Left arm','Right arm','Left leg','Right leg','Other'];
function send(cmd){
  if(!connected||!canControl)return;
  try{window.__HA.send({...cmd,request_id:++requestId});$('err').textContent='';}
  catch(e){$('err').textContent=e.message;}
}
function enable(){
  const enabled=connected&&canControl;
  for(const id of ['kinematic','dynamic','assisted','reset','zero'])$(id).disabled=!enabled;
  $('pause').disabled=!enabled||state?.mode!=='dynamic'||state?.paused;
  $('resume').disabled=!enabled||state?.mode!=='dynamic'||!state?.paused;
  for(const f of fields){f.number.disabled=!enabled||f.disabled;f.slider.disabled=!enabled||f.disabled;}
}
window.addEventListener('control-connection',e=>{
  connected=e.detail;
  if(!connected){canControl=false;$('connection').textContent='Disconnected · reload to reconnect';}
  enable();
});
window.addEventListener('control-message',async e=>{
  const msg=e.detail;
  if(msg.type==='inventory'){
    inventory=msg.inventory;canControl=msg.can_control;
    try{referenceQpos=inventory.reference_qpos||(await referenceData).reference_qpos;if(!referenceQpos)throw new Error('Reference state unavailable');}
    catch(error){$('err').textContent=error.message;return;}
    $('inventoryCount').textContent=`${inventory.joints.length} joints · ${inventory.nu} actuators`;
    buildInventory(state?.mode||'kinematic');
  }
  if(msg.type==='error'){
    $('err').textContent=msg.message;
    editingUntil=0; // Rejected Position edits must show the actual held pose.
    if(state)for(const f of fields){f.slider.value=f.read();f.number.value=f.read().toFixed(4);}
    updateMeasured();
  }
  if(msg.type==='pose'&&msg.state){
    state=msg.state;
    const uiMode=state.assisted?'assisted':state.mode;
    $('connection').textContent=`${canControl?'Controller':'Read only'} · ${uiMode} · ${state.paused?'paused':'running'} · t=${msg.sim_time.toFixed(3)} s`;
    $('modeInfo').textContent=state.assisted
      ?'Assisted mode: standing stabilizer on; gravity and contacts active. Sliders are additive torque intent (u_cmd).'
      :(state.mode==='kinematic'
        ?'Position mode: time frozen. Other joints held. Moves through collision surfaces are blocked.'
        :'Torque mode: gravity and contacts active. Press Resume to simulate.');
    $('kinematic').classList.toggle('active',state.mode==='kinematic'&&!state.assisted);
    $('dynamic').classList.toggle('active',state.mode==='dynamic'&&!state.assisted);
    $('assisted').classList.toggle('active',!!state.assisted||state.mode==='assisted');
    if(builtMode!==uiMode)buildInventory(uiMode);
    updateMeasured();enable();
  }
});
function buildInventory(mode){
  if(!inventory||!referenceQpos)return;
  builtMode=mode;kind=(mode==='dynamic'||mode==='assisted')?'actuator':'joint';
  const list=kind==='joint'?inventory.joints:inventory.actuators;
  const nav=$('inventory');nav.replaceChildren();
  for(const region of regions){
    const items=list.filter(j=>j.region===region);if(!items.length)continue;
    const d=document.createElement('details');d.open=true;
    const s=document.createElement('summary');s.textContent=`${region} · ${items.length}`;d.append(s);
    for(const j of items){
      const b=document.createElement('button');b.dataset.id=j.id;
      b.textContent=j.name;
      const sub=document.createElement('small');sub.textContent=kind==='joint'?`${j.type} joint · ${j.visual_bones.length?'mapped':'UNMAPPED'}`:`motor → ${j.joint || 'unsupported'}`;
      b.append(sub);b.onclick=()=>select(j);d.append(b);
    }nav.append(d);
  }
  select(list.find(j=>j.name===(kind==='joint'?'knee_r':'knee_r_motor'))||list[0]);
}
function jointFor(j){return kind==='joint'?j:inventory.joints[j.joint_id];}
function select(j){
  selected=j;fields=[];editingUntil=0;$('detail').scrollTop=0;
  for(const b of $('inventory').querySelectorAll('button'))b.classList.toggle('active',Number(b.dataset.id)===j.id);
  const joint=jointFor(j);
  $('selectedName').textContent=j.name;
  $('selectedMeta').textContent=kind==='joint'?`${j.type} joint · ${j.body} · local axis [${j.axis.map(v=>v.toFixed(3)).join(', ')}]`:`${state?.assisted?'Assisted u_cmd':'Torque'} actuator → ${j.joint} · gear [${j.gear.join(', ')}]`;
  $('mapping').textContent=joint?.visual_bones.length?`${joint.body} → ${joint.visual_bones.join(', ')} · mapping configured; anatomy unverified`:'UNMAPPED — no expected visual response';
  const body=inventory.bodies[joint?.body_id];
  if(body)$('selectedMeta').textContent+=` · parent: ${inventory.bodies[body.parent]?.name||'world'}`;
  $('controls').replaceChildren();
  if(kind==='actuator'){
    field(state?.assisted?'u_cmd (additive N·m; assist adds PD+COM)':'Command (motor input; unit gear = N·m)',j.ctrlrange[0],j.ctrlrange[1],0.1,()=>state?.assisted?(state?.u_cmd?.[j.id]??0):(state?.ctrl[j.id]??0),v=>send({op:'torque',id:j.id,value:v}),!j.controllable);
  }else if(j.type==='hinge'||j.type==='slide'){
    const range=j.limited?j.range:[-Math.PI,Math.PI];
    field(j.type==='hinge'?'Position (rad)':'Position (m)',...range,0.001,()=>state?.qpos[j.qpos_address]??0,v=>send({op:'joint',id:j.id,value:v}),false,referenceQpos[j.qpos_address]);
  }else{
    const isRoot=j.type==='free';
    if(isRoot)for(let i=0;i<3;i++)field(`Root ${'XYZ'[i]} (m, Z-up)`,-10,10,0.001,()=>state?.qpos[j.qpos_address+i]??0,v=>{
      const a=currentQ(j);a[i]=v;send({op:'joint',id:j.id,value:a});
    },false,referenceQpos[j.qpos_address+i]);
    const note=document.createElement('p');note.className='note';note.textContent='Rotation vector: direction is local rotation axis; length is angle in radians. The vector is bounded by the joint cone, then converted to a unit quaternion.';$('controls').append(note);
    const limit=j.limited?j.range[1]:Math.PI;
    for(let i=0;i<3;i++)field(`Rotation ${'XYZ'[i]} (rad)`,-limit,limit,0.001,()=>rotationVector(currentQ(j).slice(isRoot?3:0))[i],v=>{
      const a=currentQ(j),rv=rotationVector(a.slice(isRoot?3:0));rv[i]=v;
      let angle=Math.hypot(...rv);if(angle>limit){for(let k=0;k<3;k++)rv[k]*=limit/angle;angle=limit;}
      const q=angle<1e-10?[1,0,0,0]:[Math.cos(angle/2),...rv.map(x=>x/angle*Math.sin(angle/2))];
      send({op:'joint',id:j.id,value:isRoot?[...a.slice(0,3),...q]:q});
    },false,rotationVector(referenceQpos.slice(j.qpos_address+(isRoot?3:0),j.qpos_address+j.nq))[i]);
  }
  highlight();updateMeasured();enable();
}
function currentQ(j){return state?state.qpos.slice(j.qpos_address,j.qpos_address+j.nq):(j.type==='free'?[0,0,1,1,0,0,0]:[1,0,0,0]);}
function rotationVector(q){
  if(q[0]<0)q=q.map(x=>-x);
  const s=Math.hypot(...q.slice(1)),angle=2*Math.atan2(s,q[0]);
  return s<1e-9?[0,0,0]:q.slice(1).map(x=>x*angle/s);
}
function field(label,min,max,step,read,write,disabled=false,defaultValue=0){
  const wrap=document.createElement('div');wrap.className='field';
  const l=document.createElement('label');l.textContent=`${label} · [${min.toFixed(3)}, ${max.toFixed(3)}]`;
  const slider=document.createElement('input');slider.type='range';slider.min=min;slider.max=max;slider.step=step;slider.setAttribute('aria-label',label);
  const number=document.createElement('input');number.type='number';number.min=min;number.max=max;number.step=step;number.setAttribute('aria-label',`${label} value`);
  const update=input=>{
    const v=Number(input.value);if(!Number.isFinite(v)||v<min||v>max){$('err').textContent='Input outside allowed range';return;}
    editingUntil=performance.now()+250;slider.value=v;number.value=v;write(v);
  };
  slider.oninput=()=>update(slider);number.oninput=()=>update(number);number.onchange=()=>update(number);
  const resetDefault=()=>{
    if(slider.disabled)return;
    editingUntil=performance.now()+250;
    slider.value=defaultValue;number.value=defaultValue;write(defaultValue);
  };
  slider.title=`Double-click or double-tap to reset to ${defaultValue.toFixed(4)}`;
  slider.addEventListener('dblclick',event=>{event.preventDefault();resetDefault();});
  let tapStart=null,lastTap=null;
  slider.addEventListener('pointerdown',event=>{
    if(event.pointerType==='touch')tapStart={id:event.pointerId,x:event.clientX,y:event.clientY,time:performance.now()};
  });
  slider.addEventListener('pointerup',event=>{
    if(event.pointerType!=='touch'||tapStart?.id!==event.pointerId)return;
    const now=performance.now();
    const tap=now-tapStart.time<300&&Math.hypot(event.clientX-tapStart.x,event.clientY-tapStart.y)<12;
    tapStart=null;
    if(!tap){lastTap=null;return;}
    if(lastTap&&now-lastTap.time<350&&Math.hypot(event.clientX-lastTap.x,event.clientY-lastTap.y)<24){
      event.preventDefault();lastTap=null;resetDefault();
    }else lastTap={time:now,x:event.clientX,y:event.clientY};
  });
  slider.addEventListener('pointercancel',()=>{tapStart=null;lastTap=null;});
  wrap.append(l,slider,number);$('controls').append(wrap);fields.push({slider,number,read,disabled});
}
function updateMeasured(){
  if(!selected||!state)return;
  const j=jointFor(selected);if(!j)return;
  const q=state.qpos.slice(j.qpos_address,j.qpos_address+j.nq),v=state.qvel.slice(j.qvel_address,j.qvel_address+j.nv);
  $('measured').textContent=`qpos: ${q.map(x=>x.toFixed(5)).join(', ')}\nqvel: ${v.map(x=>x.toFixed(5)).join(', ')}${kind==='actuator'?`\nctrl: ${state.ctrl[selected.id].toFixed(4)}${state.assisted&&state.u_cmd?`\nu_cmd: ${state.u_cmd[selected.id].toFixed(4)}`:''}`:''}\nContacts: ${state.ncon}`;
  if(performance.now()>editingUntil)for(const f of fields){if(document.activeElement!==f.number){f.slider.value=f.read();f.number.value=f.read().toFixed(4);}}
  const metrics=window.__HA?.diagnostics()||[];
  $('mappingMetrics').textContent=metrics.map(m=>`${m.bone}: ${m.present&&m.mapped?`world orientation error ${m.world_error_rad.toExponential(2)} rad`:'MISSING'}${m.physical_position?' · anchor gap '+(1000*Math.hypot(m.visual_position[0]-m.physical_position[0],m.visual_position[1]-m.physical_position[2],m.visual_position[2]+m.physical_position[1])).toFixed(1)+' mm':''}`).join('\n');
  $('mappingMetrics').style.whiteSpace='pre-wrap';
}
function highlight(){if(selected)window.__HA?.select($('highlight').checked?jointFor(selected):null);}
$('highlight').onchange=highlight;
$('kinematic').onclick=()=>send({op:'mode',value:'kinematic'});
$('dynamic').onclick=()=>send({op:'mode',value:'dynamic'});
$('assisted').onclick=()=>send({op:'mode',value:'assisted'});
$('reset').onclick=()=>send({op:'reset'});
$('zero').onclick=()=>send({op:'zero'});
$('pause').onclick=()=>send({op:'pause'});
$('resume').onclick=()=>send({op:'resume'});
$('frame').onclick=()=>window.__HA?.frameCharacter();
enable();

// Picking selects an existing control; it never changes a physics-to-bone mapping.
window.addEventListener('control-pick',event=>{
  if(!inventory)return;
  const ids=event.detail.jointIds;
  const list=kind==='joint'?inventory.joints:inventory.actuators;
  const matches=list.filter(item=>ids.includes(kind==='joint'?item.id:item.joint_id));
  if(!matches.length)return;
  const item=matches.find(item=>item.id===selected?.id)||matches[0];
  select(item);
  const button=$('inventory').querySelector(`button[data-id="${item.id}"]`);
  if(button){button.closest('details').open=true;button.scrollIntoView({block:'nearest',behavior:'smooth'});}
});
