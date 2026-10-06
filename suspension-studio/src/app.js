import { importDesign, readLibrary, STORAGE } from './migration.js?v=0.2.8';
import {newDesign,preset,clone,TOPOLOGIES,title,normalizeAxle,generatePoints,changeDimensions,validateDesign,isMoving,hardpointLabel,TIRE_TREAD_WIDTH_MAX} from './model.js?v=0.2.8';
import {solveAxle} from './solver.js?v=0.2.8';
import {SuspensionViewer} from './viewer.js?v=0.2.8';
import {ARB_POINTS,newAntiRollBar,arbComplete,arbPreviewGeometry} from './arb.js?v=0.2.7';
import {OBSTACLE_TYPES,newObstacles,obstacleComplete} from './obstacles.js?v=0.2.5';
import {evaluateHeavePosition,scanHeaveTravel} from './interference.js?v=0.2.8';
const $=id=>document.getElementById(id);
let design=newDesign(),active='front',saved=[],rejected=[],libraryNotice='',storageReadable=true,bump=0,rack=0,heavePosition=0,heavePreviewActive=false,lastHeaveScan=null,toastTimer,dirty=false;
try {
  const library=readLibrary(localStorage);saved=library.saved;rejected=library.rejected;
  if(library.migrated)libraryNotice=`Converted ${library.migrated} saved design(s) to SAE J670 Z-down. Original v1 storage is retained.`;
  if(rejected.length)libraryNotice+=` ${rejected.length} unreadable design(s) retained in storage but cannot be opened.`;
} catch { storageReadable=false;libraryNotice='Saved library could not be read. Export JSON to keep new work; the stored library will not be overwritten.'; }
const viewer=new SuspensionViewer($('scene'));
document.addEventListener('studio-theme-change',()=>viewer.draw());
const escape=value=>String(value).replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
function toast(message) { $('toast').textContent=message;$('toast').classList.add('visible');clearTimeout(toastTimer);toastTimer=setTimeout(()=>$('toast').classList.remove('visible'),4200); }
function markDirty() {dirty=true;lastHeaveScan=null;$('save-state').textContent='Unsaved changes';}
function resetMotion() {bump=0;rack=0;heavePosition=0;heavePreviewActive=false;$('bump').value=0;$('steer').value=0;$('heave-position').value=0;}
function fields() {
  const a=design.axles[active];
  $('design-name').value=design.name;$('wheelbase').value=design.wheelbase;$('wheel-diameter').value=design.wheelDiameter;
  $('tire-width').value=a.tireTreadWidth??'';$('tire-width').max=TIRE_TREAD_WIDTH_MAX;$('tire-width').setCustomValidity('');
  $('heave-compression').value=design.heaveTravel.compression;$('heave-rebound').value=design.heaveTravel.rebound;
  heavePosition=Math.max(-design.heaveTravel.rebound,Math.min(design.heaveTravel.compression,heavePosition));
  $('link-od-fields').innerHTML=`<label class="field">Shared suspension link OD <div class="number-wrap"><input id="link-od" data-link-od="true" type="number" min="0.001" step="any" value="${design.interference.linkOD??''}" placeholder="mm" aria-label="Shared suspension link outer diameter"></div></label><p class="hint">Use the largest link OD in the real design. The same conservative OD is applied to suspension links, pushrods/pullrods, and bell-crank arms. Leave blank until known.</p>`;
  $('heave-position').min=-design.heaveTravel.rebound;$('heave-position').max=design.heaveTravel.compression;$('heave-position').step=.1;$('heave-position').value=heavePosition;
  $('heave-position-value').textContent=`${heavePosition.toFixed(1)} mm`;
  for(const [id,key] of Object.entries({'track':'track','rack-length':'rackLength','rack-travel':'rackTravel','spring-od':'springOD','damper-od':'damperOD','actuation':'actuation','mounting':'mounting'})) $(id).value=typeof a[key]==='number'?Number(a[key].toFixed(3)):a[key];
  $('steered').checked=a.steered;$('rack-length').disabled=!a.steered;$('rack-travel').disabled=!a.steered;
  $('actuation').disabled=a.topology==='macpherson';$('mounting').disabled=a.topology==='macpherson';
  $('compatibility-note').textContent=a.topology==='macpherson'?'MacPherson uses a direct-acting, telescopic strut.':a.actuation==='direct'?'Direct actuation connects the coilover to the upright.':'Pushrod and pullrod actuation use a bell crank to drive the coilover.';
  document.querySelectorAll('[data-axle]').forEach(b=>b.setAttribute('aria-selected',b.dataset.axle===active));
  document.querySelectorAll('[data-topology]').forEach(b=>b.setAttribute('aria-pressed',b.dataset.topology===a.topology));
  bump=Math.max(-design.heaveTravel.rebound,Math.min(design.heaveTravel.compression,bump));
  $('bump').min=-design.heaveTravel.rebound;$('bump').max=design.heaveTravel.compression;$('bump').step=.1;$('bump').value=bump;
  $('steer').min=-a.rackTravel/2;$('steer').max=a.rackTravel/2;$('steer').step=.5;$('steer').disabled=!a.steered;
  renderARB();renderObstacles();
}
function renderARB() {
  const arb=design.axles[active].antiRollBar,enabled=arb?.enabled===true;
  $('arb-enabled').checked=enabled;$('arb-enabled').setAttribute('aria-expanded',String(enabled));
  $('arb-inputs').hidden=!enabled;
  $('arb-point-fields').innerHTML=enabled?ARB_POINTS.map(({key,label,description})=>`<fieldset class="arb-point"><legend>${label}</legend><p class="hint">${description}</p><div class="arb-coordinates">${'XYZ'.split('').map((axis,i)=>`<label class="field">${axis}<input type="number" step="any" min="-10000" max="${i===1?0:10000}" value="${arb.points[key][i]??''}" placeholder="mm" data-arb-point="${key}" data-axis="${i}" aria-label="ARB ${label} ${axis}"></label>`).join('')}</div></fieldset>`).join(''):'';
}
function renderObstacles() {
  const values=design.obstacles||newObstacles();
  $('obstacle-fields').innerHTML=OBSTACLE_TYPES.map(({key,label,dimensions})=>{
    const obstacle=values[key],centerFields='XYZ'.split('').map((axis,index)=>`<label class="field">${axis}<input type="number" step="any" min="-10000" max="10000" value="${obstacle.center[index]??''}" placeholder="mm" data-obstacle-kind="${key}" data-obstacle-field="center" data-axis="${index}" aria-label="${label} center ${axis} in mm"></label>`).join('');
    const dimensionFields=dimensions.map(({key:dimension,label:dimensionLabel,axis})=>`<label class="field">${dimensionLabel} (${axis})<input type="number" step="any" min="0.001" max="10000" value="${obstacle.dimensions[dimension]??''}" placeholder="mm" data-obstacle-kind="${key}" data-obstacle-field="${dimension}" aria-label="${label} ${dimensionLabel} in mm"></label>`).join('');
    return `<fieldset class="obstacle-card"><legend>${label}</legend><div class="obstacle-card-heading"><label class="switch-label"><input type="checkbox" data-obstacle-visible="${key}" ${obstacle.visible?'checked':''} aria-label="Show ${label.toLowerCase()} in preview"> Show in preview</label></div><label class="field obstacle-reference">Coordinate reference axle<select data-obstacle-reference="${key}" aria-label="${label} coordinate reference axle"><option value="front" ${obstacle.referenceAxle==='front'?'selected':''}>Front axle datum</option><option value="rear" ${obstacle.referenceAxle==='rear'?'selected':''}>Rear axle datum</option></select></label><div class="obstacle-subheading">CENTER · SAE J670 Z-DOWN</div><div class="obstacle-coordinates">${centerFields}</div><div class="obstacle-subheading">DIMENSIONS</div><div class="obstacle-dimensions">${dimensionFields}</div><p class="hint obstacle-status" id="obstacle-${key}-status" role="status"></p></fieldset>`;
  }).join('');
}
function renderHeaveStatus(errors, current, invalidInputLabel='') {
  const target=$('heave-check-results'),linkOD=design.interference?.linkOD;
  if(invalidInputLabel) {target.innerHTML=`<p class="solver-status error">Enter a valid number for ${escape(invalidInputLabel)} before checking interference. Previous travel-scan results have been cleared.</p>`;return;}
  if(errors.length) {target.innerHTML='<p class="hint">Correct the design validation errors before running the interference check.</p>';return;}
  if(!Number.isFinite(linkOD)||linkOD<=0) {target.innerHTML='<p class="hint">Enter the shared suspension-link OD to check links and dampers.</p>';return;}
  let html=`<p class="hint heave-current" role="status">`;
  if(!current?.ok) html+=`Heave position ${heavePosition.toFixed(1)} mm could not be checked: ${escape(current?.reason||'No solved position.')}`;
  else if(current.contacts.length) html+=`<strong class="interference-warning">${current.contacts.length} modeled interference${current.contacts.length===1?'':'s'} at ${heavePosition.toFixed(1)} mm.</strong> ${current.contacts.map(item=>escape(item.target)).join('; ')}`;
  else html+=`No modeled interference at the selected ${heavePosition.toFixed(1)} mm heave position.`;
  html+='</p>';
  if(!lastHeaveScan) html+='<p class="hint">Run the full travel scan to find contact ranges. The slider checks its selected position directly.</p>';
  else if(!lastHeaveScan.ok) html+=`<p class="solver-status error">Full sweep incomplete: ${escape(lastHeaveScan.reason)}</p>`;
  else {
    html+=`<p class="hint">Full sweep completed: ${lastHeaveScan.sampleCount} solved positions, wheel-center spacing no greater than ${lastHeaveScan.sampleStepMm} mm, contact boundaries refined to ${lastHeaveScan.boundaryToleranceMm} mm. Checked ${lastHeaveScan.obstacleCount} complete visible packaging envelope${lastHeaveScan.obstacleCount===1?'':'s'}; link-to-link and link-to-damper pairs are included. Members sharing a pivot are skipped because joint and bracket solids are not defined.</p>`;
    if(lastHeaveScan.events.length) {
      html+='<div class="heave-events" role="list">'+lastHeaveScan.events.map(event=>`<button type="button" class="heave-event" data-heave-seek="${event.firstContactTravel}" role="listitem"><strong>${escape(event.target)}</strong><small>${event.startTravel.toFixed(2)} to ${event.endTravel.toFixed(2)} mm · minimum ${event.minimumClearanceMm.toFixed(2)} mm at ${event.minimumTravel.toFixed(2)} mm</small></button>`).join('')+'</div>';
    } else if(lastHeaveScan.minima.length) {
      const closest=lastHeaveScan.minima[0];
      html+=`<p class="solver-status">No interference found at sampled positions. Closest modeled pair: ${escape(closest.target)}, ${closest.clearanceMm.toFixed(2)} mm clearance at ${closest.travel.toFixed(2)} mm.</p>`;
    } else html+='<p class="solver-status">No complete link or obstacle geometry was available for a distance result.</p>';
    html+='<p class="hint">Capsule model: straight centerlines with round diameter and rounded ends. Contact is clearance ≤ 0 mm. A feature narrower than the 0.25 mm scan spacing may fall between samples; the slider evaluates each selected travel position and contacting boundaries are bracket-refined.</p>';
  }
  html+='<p class="hint">Springs, tires, uprights, anti-roll-bar geometry, and joint/bracket solids are not included.</p>';
  target.innerHTML=html;
}
function update() {
  const a=design.axles[active],errors=validateDesign(design);
  const invalidInput=[...document.querySelectorAll('input[type=number]')].find(input=>!input.disabled&&(!(input.dataset.arbPoint||input.dataset.obstacleField||input.dataset.linkOd||input.dataset.tireWidth)&&input.value===''||!input.checkValidity()));
  const invalidField=!!invalidInput,invalidInputLabel=invalidInput?(invalidInput.getAttribute('aria-label')||title(invalidInput.id||'numeric input')):'';
  if(invalidField)lastHeaveScan=null;
  const kinematicResult=errors.length||invalidField?null:solveAxle(a,bump,rack);
  const linkOD=design.interference?.linkOD;
  const heaveCurrent=!errors.length&&!invalidField&&Number.isFinite(linkOD)&&linkOD>0?evaluateHeavePosition({axle:a,design,activeAxle:active,travel:heavePosition}):null;
  const result=heavePreviewActive&&heaveCurrent?.ok?heaveCurrent.solved:kinematicResult;
  const highlightSegments=heavePreviewActive&&heaveCurrent?.ok?new Set(heaveCurrent.contacts.flatMap(item=>item.memberIds)):new Set();
  viewer.update(a,design.wheelDiameter,result,{obstacles:design.obstacles||{},activeAxle:active,wheelbase:design.wheelbase,linkOD,highlightSegments});
  $('interference-legend').hidden=!highlightSegments.size;
  $('run-heave-check').disabled=errors.length>0||invalidField||!Number.isFinite(linkOD)||linkOD<=0;
  $('fit-all').setAttribute('aria-pressed',String(viewer.fitMode));if(viewer.fitMode)document.querySelectorAll('[data-view]').forEach(button=>button.setAttribute('aria-pressed','false'));
  $('viewer-axle').textContent=`${active.toUpperCase()} AXLE`;$('viewer-topology').textContent=TOPOLOGIES[a.topology].name;$('viewer-detail').textContent=invalidField?'Invalid input · Last accepted static geometry':`${title(a.actuation)} · ${title(a.mounting)}`;
  $('stat-track').innerHTML=`${a.track.toLocaleString(undefined,{maximumFractionDigits:1})} <span>mm</span>`;
  $('stat-points').textContent=Object.keys(a.hardpoints).length;$('stat-solver').textContent=TOPOLOGIES[a.topology].solver;
  $('tire-viewer-note').textContent=a.tireTreadWidth>0?`${a.tireTreadWidth} mm tread width · Ideal circular tread band. Tire section width and sidewall profile are not defined.`:'Tread width unentered · Showing the outer-diameter circle only; no tread width is assumed.';
  const arbEnabled=a.antiRollBar?.enabled===true,arbReady=arbComplete(a.antiRollBar),arbPreview=arbPreviewGeometry(a.antiRollBar);
  const invalidArb=[...$('arb-point-fields').querySelectorAll('input')].find(input=>!input.validity.valid);
  $('arb-status').classList.toggle('error',!!invalidArb);
  $('arb-status').textContent=invalidArb?'Use coordinates from −10000 to 10000 mm; left-side Y must be zero or negative. Only complete valid XYZ points are previewed.':arbReady?'All ARB points entered and shown.':arbPreview.layoutComplete?'U-bar and both drop links shown. Chassis bearing location is unentered; its marker is optional for this preview and is not inferred.':arbPreview.enteredPointCount?`${arbPreview.enteredPointCount} of 4 left-side points shown, with their mirrored right-side points. Unentered: ${arbPreview.missingLabels.join(', ')}. Segments appear when both endpoints are entered; unknown coordinates stay blank.`:'Enter XYZ for any ARB point to preview it. The bar bend, lever-arm tip, and suspension pickup define the displayed bar and links; the bearing marker is optional. A checkbox alone does not define a location.';
  $('arb-legend').hidden=!arbPreview.points.length;
  $('arb-viewer-note').hidden=!arbEnabled;
  $('arb-viewer-note').textContent=arbReady?'U-bar ARB · static reference, including during bump and steering.':arbPreview.layoutComplete?'U-bar ARB · static bar and drop links. Bearing location unentered; no bearing marker inferred.':arbPreview.points.length?`U-bar ARB · partial static layout (${arbPreview.enteredPointCount}/4 left-side points). Unentered: ${arbPreview.missingLabels.join(', ')}. No missing geometry is inferred.`:'U-bar ARB · no complete XYZ points entered yet.';
  let hasVisibleObstacle=false;
  for(const {key,label} of OBSTACLE_TYPES) {
    const obstacle=design.obstacles?.[key],visible=obstacle?.visible===true,ready=obstacleComplete(obstacle,key);
    const invalid=[...$('obstacle-fields').querySelectorAll(`[data-obstacle-kind="${key}"]`)].some(input=>input.type==='number'&&!input.checkValidity());
    $(`${key}-legend`).hidden=!visible||!ready;
    const status=$(`obstacle-${key}-status`);status.classList.toggle('error',invalid);
    status.textContent=invalid?'Enter a center coordinate from −10000 to 10000 mm and dimensions greater than 0 up to 10000 mm.':!visible?'Hidden from the preview and heave check; entered values are retained.':ready?'Shown in both axle views and included in heave clearance checks.':'Enter all center coordinates and dimensions to show and check this envelope; incomplete geometry can be saved.';
    hasVisibleObstacle||=visible;
  }
  $('obstacle-viewer-note').hidden=!hasVisibleObstacle;
  $('obstacle-viewer-note').textContent='Packaging envelopes use SAE J670 Z-down millimeters and remain static during suspension motion. Complete envelopes with “Show in preview” enabled are included in the heave interference check.';
  $('axle-summary').innerHTML=Object.entries(design.axles).map(([name,axle])=>`<div class="summary-row"><span class="axle-icon">${name==='front'?'F':'R'}</span><div><strong>${title(name)} axle · ${TOPOLOGIES[axle.topology].name}</strong><small>${title(axle.actuation)} / ${title(axle.mounting)} · ${axle.steered?'Steered':'Fixed toe links'}</small></div><span>${Number(axle.track.toFixed(1))} mm</span></div>`).join('');
  $('bump-value').textContent=`${bump} mm`;$('steer-value').textContent=`${rack} mm`;$('heave-position-value').textContent=`${heavePosition.toFixed(1)} mm`;
  if(invalidField) $('solver-results').innerHTML=`<p class="solver-status error">Enter a valid number for ${escape(invalidInputLabel)} before solving motion. The preview shows static geometry from the last accepted values.</p>`;
  else if(errors.length) $('solver-results').innerHTML=`<p class="solver-status error">${errors.map(escape).join('<br>')}</p>`;
  else if(!result.ok) $('solver-results').innerHTML=`<p class="solver-status error">${escape(result.left?.reason||result.right?.reason||'No nearby assembly solution.')} Showing static geometry.</p>`;
  else $('solver-results').innerHTML=`<div class="solver-metrics"><span>Left Δ camber <strong>${result.left.camber.toFixed(2)}°</strong></span><span>Left Δ toe-in <strong>${result.left.toe.toFixed(2)}°</strong></span><span>Right Δ toe-in <strong>${result.right.toe.toFixed(2)}°</strong></span><span>Left / right steer <strong>${result.left.steer.toFixed(2)}° / ${result.right.steer.toFixed(2)}°</strong></span><span>Left damper compression <strong>${result.left.compression.toFixed(2)} mm</strong></span></div><p class="solver-status">● Position solved · maximum constraint residual ${Math.max(result.left.error,result.right.error).toFixed(5)} mm</p>`;
  renderHeaveStatus(errors,heaveCurrent,invalidInputLabel);
  $('save').disabled=!storageReadable||errors.length>0||invalidField;$('export').disabled=errors.length>0||invalidField;
  if(!$('hardpoints-panel').hidden && !document.activeElement?.dataset.point) renderHardpoints();
}
function renderHardpoints() {
  $('hardpoint-axle').textContent=`/ ${title(active)} axle`;
  $('hardpoint-rows').innerHTML=Object.entries(design.axles[active].hardpoints).map(([key,xyz])=>`<tr><td>${hardpointLabel(key)}</td>${xyz.map((n,i)=>`<td><input type="number" step="0.01" min="-10000" max="10000" value="${Number(n.toFixed(2))}" data-point="${key}" data-axis="${i}" aria-label="${hardpointLabel(key)} ${'XYZ'[i]}"></td>`).join('')}<td>${isMoving(key)?'Upright':key.startsWith('rocker_')&&key!=='rocker_pivot'?'Rocker':'Chassis'}</td></tr>`).join('');
}
function reveal(panel) {
  $(panel).hidden=false;if(panel==='hardpoints-panel')renderHardpoints();$(panel).scrollIntoView({behavior:'smooth',block:'start'});
}
function geometryChange(change) {
  const a=design.axles[active], customized=JSON.stringify(a.hardpoints)!==JSON.stringify(generatePoints(a,design.wheelDiameter));
  if(customized&&!confirm('Changing topology or actuation replaces this axle’s current hardpoints with generated geometry. Continue?')) {fields();return;}
  change(a);normalizeAxle(a);a.hardpoints=generatePoints(a,design.wheelDiameter);resetMotion();markDirty();fields();update();
}
for(const button of document.querySelectorAll('[data-axle]'))button.addEventListener('click',()=>{active=button.dataset.axle;lastHeaveScan=null;resetMotion();fields();update();});
for(const button of document.querySelectorAll('[data-topology]'))button.addEventListener('click',()=>{if(button.dataset.topology!==design.axles[active].topology)geometryChange(a=>a.topology=button.dataset.topology);});
$('actuation').addEventListener('change',()=>geometryChange(a=>a.actuation=$('actuation').value));
$('mounting').addEventListener('change',()=>geometryChange(a=>{a.mounting=$('mounting').value;a.actuation=a.mounting==='direct'?'direct':a.actuation==='direct'?'pushrod':a.actuation;}));
function validInput(input) {if(input.value.trim()===''||!input.checkValidity()){update();return false;}return true;}
for(const [id,key] of Object.entries({'track':'track','rack-length':'rackLength','rack-travel':'rackTravel','spring-od':'springOD','damper-od':'damperOD'})) $(id).addEventListener('input',()=>{
  const input=$(id);if(!validInput(input))return;
  const a=design.axles[active],backup=clone(a);changeDimensions(a,key,Number(input.value));
  const errors=validateDesign(design);if(errors.length){design.axles[active]=backup;fields();update();toast(errors[0]);return;}
  resetMotion();markDirty();update();
  if(key==='track')$('rack-length').value=Number(a.rackLength.toFixed(3));
});
$('steered').addEventListener('change',()=>{design.axles[active].steered=$('steered').checked;resetMotion();markDirty();fields();update();});
$('tire-width').addEventListener('input',()=>{
  const input=$('tire-width');input.setCustomValidity(input.value!==''&&Number(input.value)<=0?'Tire tread width must be greater than 0 mm.':'');
  markDirty();if(!input.checkValidity()){update();return;}
  design.axles[active].tireTreadWidth=input.value===''?null:Number(input.value);update();
});
$('arb-enabled').addEventListener('change',()=>{
  const a=design.axles[active];a.antiRollBar??=newAntiRollBar();a.antiRollBar.enabled=$('arb-enabled').checked;
  markDirty();renderARB();update();
});
$('arb-point-fields').addEventListener('input',e=>{
  const input=e.target;if(!input.dataset.arbPoint)return;
  if(!input.checkValidity()){update();return;}
  const arb=design.axles[active].antiRollBar;
  arb.points[input.dataset.arbPoint][Number(input.dataset.axis)]=input.value===''?null:Number(input.value);
  markDirty();update();
});
$('obstacle-fields').addEventListener('change',e=>{
  const input=e.target;
  if(input.dataset.obstacleVisible){
    const collection=design.obstacles??=newObstacles();collection[input.dataset.obstacleVisible].visible=input.checked;markDirty();update();return;
  }
  if(input.dataset.obstacleReference){
    const collection=design.obstacles??=newObstacles();collection[input.dataset.obstacleReference].referenceAxle=input.value;markDirty();update();
  }
});
$('obstacle-fields').addEventListener('input',e=>{
  const input=e.target;if(!input.dataset.obstacleField)return;
  markDirty();if(!input.checkValidity()){update();return;}
  const collection=design.obstacles??=newObstacles(),obstacle=collection[input.dataset.obstacleKind],field=input.dataset.obstacleField,value=input.value===''?null:Number(input.value);
  if(field==='center')obstacle.center[Number(input.dataset.axis)]=value;else obstacle.dimensions[field]=value;
  update();
});
$('link-od-fields').addEventListener('input',e=>{
  const input=e.target;if(input.id!=='link-od')return;
  markDirty();if(input.value!==''&&!input.checkValidity()){update();return;}
  design.interference.linkOD=input.value===''?null:Number(input.value);update();
});
$('design-name').addEventListener('input',()=>{design.name=$('design-name').value;markDirty();update();});
$('wheelbase').addEventListener('input',()=>{if(!validInput($('wheelbase')))return;design.wheelbase=Number($('wheelbase').value);markDirty();update();});
$('wheel-diameter').addEventListener('input',()=>{
  if(!validInput($('wheel-diameter')))return;const diameter=Number($('wheel-diameter').value),dz=(diameter-design.wheelDiameter)/2;
  for(const a of Object.values(design.axles))for(const p of Object.values(a.hardpoints))p[2]-=dz;
  for(const a of Object.values(design.axles))if(a.antiRollBar)for(const p of Object.values(a.antiRollBar.points))if(p[2]!==null)p[2]-=dz;
  if(design.obstacles)for(const obstacle of Object.values(design.obstacles))if(obstacle.center[2]!==null)obstacle.center[2]-=dz;
  design.wheelDiameter=diameter;resetMotion();markDirty();renderARB();renderObstacles();update();
});
$('hardpoint-rows').addEventListener('input',e=>{
  const input=e.target;if(!input.dataset.point||!validInput(input))return;
  const a=design.axles[active],backup=clone(a),key=input.dataset.point,axis=Number(input.dataset.axis);
  a.hardpoints[key][axis]=Number(input.value);
  if(key==='wheel_center'&&axis===1)a.track=-Number(input.value)*2;
  if(key==='tie_rod_inner'&&axis===1)a.rackLength=-Number(input.value)*2;
  const errors=validateDesign(design);if(errors.length){design.axles[active]=backup;renderHardpoints();update();toast(errors[0]);return;}
  resetMotion();markDirty();update();
  $('track').value=a.track;$('rack-length').value=Number(a.rackLength.toFixed(3));
});
$('regenerate').addEventListener('click',()=>{if(!confirm(`Replace ${active} axle hardpoints with generated defaults?`))return;const a=design.axles[active];a.hardpoints=generatePoints(a,design.wheelDiameter);resetMotion();markDirty();update();toast('Axle hardpoints regenerated.');});
for(const id of ['edit-hardpoints','hardpoints-step'])$(id).addEventListener('click',()=>reveal('hardpoints-panel'));
$('kinematics-step').addEventListener('click',()=>reveal('kinematics-panel'));
$('bump').addEventListener('input',()=>{bump=Number($('bump').value);heavePosition=Math.max(-design.heaveTravel.rebound,Math.min(design.heaveTravel.compression,bump));$('heave-position').value=heavePosition;heavePreviewActive=false;update();});
$('steer').addEventListener('input',()=>{rack=Number($('steer').value);heavePreviewActive=false;update();});
for(const [id,key] of Object.entries({'heave-compression':'compression','heave-rebound':'rebound'})) $(id).addEventListener('input',()=>{
  const input=$(id);if(!validInput(input))return;
  design.heaveTravel[key]=Number(input.value);heavePosition=Math.max(-design.heaveTravel.rebound,Math.min(design.heaveTravel.compression,heavePosition));
  bump=Math.max(-design.heaveTravel.rebound,Math.min(design.heaveTravel.compression,bump));
  $('heave-position').min=-design.heaveTravel.rebound;$('heave-position').max=design.heaveTravel.compression;$('heave-position').value=heavePosition;
  $('bump').min=-design.heaveTravel.rebound;$('bump').max=design.heaveTravel.compression;$('bump').value=bump;
  markDirty();update();
});
$('heave-position').addEventListener('input',()=>{
  heavePosition=Number($('heave-position').value);heavePreviewActive=true;bump=heavePosition;rack=0;
  $('bump').value=bump;$('steer').value=0;update();
});
$('run-heave-check').addEventListener('click',()=>{
  heavePreviewActive=true;bump=heavePosition;rack=0;$('bump').value=bump;$('steer').value=0;
  lastHeaveScan=scanHeaveTravel({axle:design.axles[active],design,activeAxle:active});update();
});
$('heave-check-results').addEventListener('click',e=>{
  const button=e.target.closest('[data-heave-seek]');if(!button)return;
  heavePosition=Number(button.dataset.heaveSeek);heavePreviewActive=true;bump=heavePosition;rack=0;
  $('heave-position').value=heavePosition;$('bump').value=bump;$('steer').value=0;update();
});
$('reset-motion').addEventListener('click',()=>{resetMotion();update();});
for(const button of document.querySelectorAll('[data-view]'))button.addEventListener('click',()=>{viewer.view(button.dataset.view);document.querySelectorAll('[data-view]').forEach(b=>b.setAttribute('aria-pressed',b===button));$('fit-all').setAttribute('aria-pressed','false');});
$('fit-all').addEventListener('click',()=>{viewer.fitAll();$('fit-all').setAttribute('aria-pressed','true');document.querySelectorAll('[data-view]').forEach(b=>b.setAttribute('aria-pressed','false'));});
$('reset-view').addEventListener('click',()=>{viewer.view('iso');document.querySelectorAll('[data-view]').forEach(b=>b.setAttribute('aria-pressed',b.dataset.view==='iso'));$('fit-all').setAttribute('aria-pressed','false');});
$('show-labels').addEventListener('change',()=>{viewer.labels=$('show-labels').checked;viewer.draw();});
function load(d) {
  let imported;try{imported=importDesign(d);}catch(error){$('library-error').textContent=error.message;return;}
  design=imported.design;active='front';resetMotion();dirty=false;fields();update();$('save-state').textContent='Design loaded';$('library').close();toast(imported.migrated?`Converted ${design.name} to SAE J670 Z-down; save or export to keep v2.`:`Opened ${design.name}`);
}
function showLibrary() {
  $('library-error').textContent=libraryNotice;
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
  if(!storageReadable){toast(libraryNotice);return;}
  const errors=validateDesign(design);if(errors.length){toast(errors[0]);return;}
  const next=clone(saved),index=next.findIndex(d=>d.name===design.name);
  if(index>=0)next[index]=clone(design);else next.push(clone(design));
  try{localStorage.setItem(STORAGE,JSON.stringify([...next,...rejected]));saved=next;dirty=false;$('save-state').textContent='Saved locally';toast('Design saved in this browser.');}catch{toast('Browser storage is unavailable or full. Export JSON to keep your design.');}
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
if(libraryNotice)toast(libraryNotice);
