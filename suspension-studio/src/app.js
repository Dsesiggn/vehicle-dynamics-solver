import {newDesign,preset,clone,TOPOLOGIES,title,normalizeAxle,generatePoints,changeDimensions,validateDesign,isMoving} from './model.js';
import {solveAxle} from './solver.js';
import {SuspensionViewer} from './viewer.js';
const $=id=>document.getElementById(id), STORAGE='suspension-studio.designs.v1';
let design=newDesign(),active='front',saved=[],bump=0,rack=0,toastTimer,dirty=false;
try { const value=JSON.parse(localStorage.getItem(STORAGE)||'[]'); if(Array.isArray(value)) saved=value.filter(d=>validateDesign(d).length===0); } catch { /* App remains usable if storage is unavailable. */ }
const viewer=new SuspensionViewer($('scene'));
const escape=value=>String(value).replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
function toast(message) { $('toast').textContent=message;$('toast').classList.add('visible');clearTimeout(toastTimer);toastTimer=setTimeout(()=>$('toast').classList.remove('visible'),4200); }
function markDirty() {dirty=true;$('save-state').textContent='Unsaved changes';}
function resetMotion() {bump=0;rack=0;$('bump').value=0;$('steer').value=0;}
function fields() {
  const a=design.axles[active];
  $('design-name').value=design.name;$('wheelbase').value=design.wheelbase;$('wheel-diameter').value=design.wheelDiameter;
  for(const [id,key] of Object.entries({'track':'track','rack-length':'rackLength','rack-travel':'rackTravel','spring-od':'springOD','damper-od':'damperOD','actuation':'actuation','mounting':'mounting'})) $(id).value=typeof a[key]==='number'?Number(a[key].toFixed(3)):a[key];
  $('steered').checked=a.steered;$('rack-length').disabled=!a.steered;$('rack-travel').disabled=!a.steered;
  $('actuation').disabled=a.topology==='macpherson';$('mounting').disabled=a.topology==='macpherson';
  $('compatibility-note').textContent=a.topology==='macpherson'?'MacPherson uses a direct-acting, telescopic strut.':a.actuation==='direct'?'Direct actuation connects the coilover to the upright.':'Pushrod and pullrod actuation use a bell crank to drive the coilover.';
  document.querySelectorAll('[data-axle]').forEach(b=>b.setAttribute('aria-selected',b.dataset.axle===active));
  document.querySelectorAll('[data-topology]').forEach(b=>b.setAttribute('aria-pressed',b.dataset.topology===a.topology));
  $('steer').min=-a.rackTravel/2;$('steer').max=a.rackTravel/2;$('steer').step=.5;$('steer').disabled=!a.steered;
}
function update() {
  const a=design.axles[active],errors=validateDesign(design),result=errors.length?null:solveAxle(a,bump,rack);
  viewer.update(a,design.wheelDiameter,result);
  $('viewer-axle').textContent=`${active.toUpperCase()} AXLE`;$('viewer-topology').textContent=TOPOLOGIES[a.topology].name;$('viewer-detail').textContent=`${title(a.actuation)} · ${title(a.mounting)}`;
  $('stat-track').innerHTML=`${a.track.toLocaleString(undefined,{maximumFractionDigits:1})} <span>mm</span>`;
  $('stat-points').textContent=Object.keys(a.hardpoints).length;$('stat-solver').textContent=TOPOLOGIES[a.topology].solver;
  $('axle-summary').innerHTML=Object.entries(design.axles).map(([name,axle])=>`<div class="summary-row"><span class="axle-icon">${name==='front'?'F':'R'}</span><div><strong>${title(name)} axle · ${TOPOLOGIES[axle.topology].name}</strong><small>${title(axle.actuation)} / ${title(axle.mounting)} · ${axle.steered?'Steered':'Fixed toe links'}</small></div><span>${Number(axle.track.toFixed(1))} mm</span></div>`).join('');
  $('bump-value').textContent=`${bump} mm`;$('steer-value').textContent=`${rack} mm`;
  if(errors.length) $('solver-results').innerHTML=`<p class="solver-status error">${errors.map(escape).join('<br>')}</p>`;
  else if(!result.ok) $('solver-results').innerHTML=`<p class="solver-status error">${escape(result.left.reason||result.right.reason)} Showing static geometry.</p>`;
  else $('solver-results').innerHTML=`<div class="solver-metrics"><span>Left Δ camber <strong>${result.left.camber.toFixed(2)}°</strong></span><span>Left Δ toe <strong>${result.left.toe.toFixed(2)}°</strong></span><span>Right Δ toe <strong>${result.right.toe.toFixed(2)}°</strong></span><span>Left damper compression <strong>${result.left.compression.toFixed(2)} mm</strong></span></div><p class="solver-status">● Position solved · maximum constraint residual ${Math.max(result.left.error,result.right.error).toFixed(5)} mm</p>`;
  const invalidField=[...document.querySelectorAll('input[type=number]')].some(input=>input.value===''||!input.checkValidity());
  $('save').disabled=errors.length>0||invalidField;$('export').disabled=errors.length>0||invalidField;
  if(!$('hardpoints-panel').hidden && !document.activeElement?.dataset.point) renderHardpoints();
}
function renderHardpoints() {
  $('hardpoint-axle').textContent=`/ ${title(active)} axle`;
  $('hardpoint-rows').innerHTML=Object.entries(design.axles[active].hardpoints).map(([key,xyz])=>`<tr><td>${title(key.replaceAll('_',' '))}</td>${xyz.map((n,i)=>`<td><input type="number" step="0.01" min="-10000" max="10000" value="${Number(n.toFixed(2))}" data-point="${key}" data-axis="${i}" aria-label="${key} ${'XYZ'[i]}"></td>`).join('')}<td>${isMoving(key)?'Upright':key.startsWith('rocker_')&&key!=='rocker_pivot'?'Rocker':'Chassis'}</td></tr>`).join('');
}
function reveal(panel) {
  $(panel).hidden=false;if(panel==='hardpoints-panel')renderHardpoints();$(panel).scrollIntoView({behavior:'smooth',block:'start'});
}
function geometryChange(change) {
  const a=design.axles[active], customized=JSON.stringify(a.hardpoints)!==JSON.stringify(generatePoints(a,design.wheelDiameter));
  if(customized&&!confirm('Changing topology or actuation replaces this axle’s current hardpoints with generated geometry. Continue?')) {fields();return;}
  change(a);normalizeAxle(a);a.hardpoints=generatePoints(a,design.wheelDiameter);resetMotion();markDirty();fields();update();
}
for(const button of document.querySelectorAll('[data-axle]'))button.addEventListener('click',()=>{active=button.dataset.axle;resetMotion();fields();update();});
for(const button of document.querySelectorAll('[data-topology]'))button.addEventListener('click',()=>{if(button.dataset.topology!==design.axles[active].topology)geometryChange(a=>a.topology=button.dataset.topology);});
$('actuation').addEventListener('change',()=>geometryChange(a=>a.actuation=$('actuation').value));
$('mounting').addEventListener('change',()=>geometryChange(a=>{a.mounting=$('mounting').value;a.actuation=a.mounting==='direct'?'direct':a.actuation==='direct'?'pushrod':a.actuation;}));
function validInput(input) {if(input.value.trim()===''||!input.checkValidity()){$('save').disabled=true;$('export').disabled=true;return false;}return true;}
for(const [id,key] of Object.entries({'track':'track','rack-length':'rackLength','rack-travel':'rackTravel','spring-od':'springOD','damper-od':'damperOD'})) $(id).addEventListener('input',()=>{
  const input=$(id);if(!validInput(input))return;
  const a=design.axles[active],backup=clone(a);changeDimensions(a,key,Number(input.value));
  const errors=validateDesign(design);if(errors.length){design.axles[active]=backup;fields();toast(errors[0]);return;}
  resetMotion();markDirty();update();
  if(key==='track')$('rack-length').value=Number(a.rackLength.toFixed(3));
});
$('steered').addEventListener('change',()=>{design.axles[active].steered=$('steered').checked;resetMotion();markDirty();fields();update();});
$('design-name').addEventListener('input',()=>{design.name=$('design-name').value;markDirty();update();});
$('wheelbase').addEventListener('input',()=>{if(!validInput($('wheelbase')))return;design.wheelbase=Number($('wheelbase').value);markDirty();update();});
$('wheel-diameter').addEventListener('input',()=>{
  if(!validInput($('wheel-diameter')))return;const diameter=Number($('wheel-diameter').value),dz=(diameter-design.wheelDiameter)/2;
  for(const a of Object.values(design.axles))for(const p of Object.values(a.hardpoints))p[2]+=dz;
  design.wheelDiameter=diameter;resetMotion();markDirty();update();
});
$('hardpoint-rows').addEventListener('input',e=>{
  const input=e.target;if(!input.dataset.point||!validInput(input))return;
  const a=design.axles[active],backup=clone(a),key=input.dataset.point,axis=Number(input.dataset.axis);
  a.hardpoints[key][axis]=Number(input.value);
  if(key==='wheel_center'&&axis===0)a.track=Number(input.value)*2;
  if(key==='tie_rod_inner'&&axis===0)a.rackLength=Number(input.value)*2;
  const errors=validateDesign(design);if(errors.length){design.axles[active]=backup;renderHardpoints();toast(errors[0]);return;}
  resetMotion();markDirty();update();
  $('track').value=a.track;$('rack-length').value=Number(a.rackLength.toFixed(3));
});
$('regenerate').addEventListener('click',()=>{if(!confirm(`Replace ${active} axle hardpoints with generated defaults?`))return;const a=design.axles[active];a.hardpoints=generatePoints(a,design.wheelDiameter);resetMotion();markDirty();update();toast('Axle hardpoints regenerated.');});
for(const id of ['edit-hardpoints','hardpoints-step'])$(id).addEventListener('click',()=>reveal('hardpoints-panel'));
$('kinematics-step').addEventListener('click',()=>reveal('kinematics-panel'));
$('bump').addEventListener('input',()=>{bump=Number($('bump').value);update();});$('steer').addEventListener('input',()=>{rack=Number($('steer').value);update();});
$('reset-motion').addEventListener('click',()=>{resetMotion();update();});
for(const button of document.querySelectorAll('[data-view]'))button.addEventListener('click',()=>{viewer.view(button.dataset.view);document.querySelectorAll('[data-view]').forEach(b=>b.setAttribute('aria-pressed',b===button));});
$('reset-view').addEventListener('click',()=>{viewer.view('iso');document.querySelectorAll('[data-view]').forEach(b=>b.setAttribute('aria-pressed',b.dataset.view==='iso'));});
$('show-labels').addEventListener('change',()=>{viewer.labels=$('show-labels').checked;viewer.draw();});
function load(d) {
  const errors=validateDesign(d);if(errors.length){$('library-error').textContent=errors.join(' ');return;}
  design=clone(d);active='front';resetMotion();dirty=false;fields();update();$('save-state').textContent='Design loaded';$('library').close();toast(`Opened ${design.name}`);
}
function showLibrary() {
  $('library-error').textContent='';
  $('presets').innerHTML=[['formula','Formula / double wishbone','1200 mm front track · Pushrod · Generated starting geometry'],['road','Road car / strut + multi-link','MacPherson front · Five-link rear · Generated starting geometry'],['cr26','CR26 / original front geometry','Original FS-CR26.py front hardpoints · Rear and actuation geometry are generated; wheelbase is assumed.']].map(([id,name,description])=>`<button class="library-item" data-preset="${id}"><strong>${name} ↗</strong><small>${description}</small></button>`).join('');
  $('saved-designs').innerHTML=saved.length?saved.map((d,i)=>`<button class="library-item" data-saved="${i}"><strong>${escape(d.name)} ↗</strong><small>${TOPOLOGIES[d.axles.front.topology].name} / ${TOPOLOGIES[d.axles.rear.topology].name} · ${d.wheelbase} mm wheelbase</small></button>`).join(''):'<p class="hint">No saved designs yet. Save your first configuration to find it here.</p>';
  $('library').showModal();
}
for(const id of ['existing-design','library-nav'])$(id).addEventListener('click',showLibrary);
$('close-library').addEventListener('click',()=>$('library').close());
$('presets').addEventListener('click',e=>{const b=e.target.closest('[data-preset]');if(b)load(preset(b.dataset.preset));});
$('saved-designs').addEventListener('click',e=>{const b=e.target.closest('[data-saved]');if(b)load(saved[Number(b.dataset.saved)]);});
$('new-design').addEventListener('click',()=>{if(dirty&&!confirm('Start a new design and discard the current unsaved draft?'))return;load(newDesign());$('save-state').textContent='Unsaved design';});
$('save').addEventListener('click',()=>{
  const errors=validateDesign(design);if(errors.length){toast(errors[0]);return;}
  const next=clone(saved),index=next.findIndex(d=>d.name===design.name);
  if(index>=0)next[index]=clone(design);else next.push(clone(design));
  try{localStorage.setItem(STORAGE,JSON.stringify(next));saved=next;dirty=false;$('save-state').textContent='Saved locally';toast('Design saved in this browser.');}catch{toast('Browser storage is unavailable or full. Export JSON to keep your design.');}
});
$('export').addEventListener('click',()=>{
  if(validateDesign(design).length)return;
  const blob=new Blob([JSON.stringify(design,null,2)],{type:'application/json'}),url=URL.createObjectURL(blob),a=document.createElement('a');
  a.href=url;a.download=(design.name.replace(/[^a-z0-9]+/gi,'-').toLowerCase()||'suspension')+'.json';a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);toast('Design JSON exported.');
});
$('import-file').addEventListener('change',async e=>{
  const file=e.target.files[0];if(!file)return;
  try{if(file.size>1000000)throw Error('Design files must be smaller than 1 MB.');load(JSON.parse(await file.text()));}catch(error){$('library-error').textContent=error instanceof SyntaxError?'The file is not valid JSON.':error.message;}finally{e.target.value='';}
});
window.addEventListener('beforeunload',e=>{if(dirty){e.preventDefault();e.returnValue='';}});
fields();update();
