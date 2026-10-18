import {newSteeringDesign,exampleSteeringDesign,validateSteeringDesign} from './steering.js';
import {SteeringViewer,SteeringChart} from './steering-viewer.js';
import {rackLongitudinalOffset,setRackLongitudinalOffset} from './steering-rack.js';
import {createSteeringSweepLoader} from './steering-api.js';

const $=id=>document.getElementById(id),STORAGE='suspension-studio.steering.v1';
const sides=['left','right'],pointNames={axisUpper:'Upper steering-axis point',axisLower:'Lower steering-axis point',tieInner:'Rack / tie-rod inner joint',tieOuter:'Upright / tie-rod outer joint'};
const scalarNames={wheelCenterZ:'Nominal wheel center Z',staticToeDeg:'Static toe-in',staticCamberDeg:'Static camber'};
const escape=value=>String(value).replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
const format=(value,digits=2)=>Number.isFinite(value)?value.toFixed(digits):'—';
const signed=(value,digits=2)=>Number.isFinite(value)?`${value>0?'+':''}${value.toFixed(digits)}`:'—';
let design=newSteeringDesign(),angle=0,rows=[],dirty=false,ready=false,storageReadable=true,exportUrl=null,currentConstruction=null,rackOffsetPending=false,eligible=false,engineState='idle',engineError=null;
const planViewer=new SteeringViewer($('steering-plan-scene')),chart=new SteeringChart($('steering-chart'));
let spatialViewer=null,viewer=planViewer,viewMode='plan';
try{
  const {SteeringViewer3D}=await import('./steering-viewer-3d.js');
  spatialViewer=new SteeringViewer3D($('steering-scene'));
  viewer=spatialViewer;viewMode='3d';viewer.frontOnly=true;
  spatialViewer.onViewChange=name=>{$('steering-camera').value=name;syncViewDescription();};
  spatialViewer.onUnavailable=message=>{
    $('steering-viewer-notice').textContent=`3D rendering is unavailable: ${message} The labeled 2D construction view remains available; Python calculations are unchanged.`;
    $('steering-viewer-notice').hidden=false;$('steering-mode-3d').disabled=true;setViewMode('plan');
  };
}catch(error){
  $('steering-viewer-notice').textContent=`3D rendering could not start: ${error.message}. The labeled 2D construction view remains available; Python calculations are unchanged.`;
  $('steering-viewer-notice').hidden=false;$('steering-mode-3d').disabled=true;
}
viewer.showArmLines=$('arm-lines-overlay').checked;viewer.showArmAngles=$('arm-angles-overlay').checked;viewer.fitIntersection=false;
const sweepLoader=createSteeringSweepLoader({onState:receiveSweepState});

function syncViewDescription(){
  const spatial=viewMode==='3d';
  $('steering-scene').hidden=!spatial;$('steering-plan-scene').hidden=spatial;
  spatialViewer?.setVisible(spatial);
  $('steering-mode-3d').setAttribute('aria-pressed',String(spatial));$('steering-mode-plan').setAttribute('aria-pressed',String(!spatial));
  $('steering-camera-controls').hidden=!spatial;
  $('steering-view-description').textContent=spatial?'3D hardpoints and wheel-plane orientation · SAE +X forward, +Y right, +Z down':'2D construction · orthographic X–Y joint projection · +X forward ↑ · +Y right →';
  $('steering-interaction-help').hidden=!spatial;
  $('arm-lines-label').textContent=spatial?'Plan arm projections':'Arm extensions';
  $('arm-angles-label').textContent=spatial?'Angles α / β':'Plan angles α';
  $('steering-graphic-note').textContent=spatial?'Solid linkage follows the actual XYZ joints. Effective arms connect each outer joint to its perpendicular steering-axis foot C. Dashed plan constructions lie on Z = 0. Wheel-plane rings and link thickness are symbolic; tire dimensions and physical arm shapes are not defined. Rear markers show the nominal axle datum, not rear-wheel height.':'Joint positions are projected from the same 3D solution. Plan arm lines join axis point K at outer-ball height to the outer joint. Arcs show α; wheel icons use heading only and do not show camber or tire shape.';
  $('steering-view').setAttribute('aria-pressed',String(viewer.frontOnly));$('steering-view').textContent=viewer.frontOnly?'Show full vehicle':'Focus front linkage';
}
function setViewMode(mode){
  if(mode==='3d'&&!spatialViewer)return;
  const next=mode==='3d'?spatialViewer:planViewer;
  if(next!==viewer)for(const key of ['frontOnly','showIdeal','showArmLines','showArmAngles','fitIntersection'])next[key]=viewer[key];
  viewer=next;viewMode=mode;syncViewDescription();showPosition();
  if(mode==='3d'){$('steering-camera').value=spatialViewer.viewName;spatialViewer.fit();}
}

function notify(message,error=false){$('steering-notice').textContent=message;$('steering-notice').classList.toggle('error',error);$('steering-notice').hidden=false;}
function markDirty(){dirty=true;$('steering-save-state').textContent='Unsaved changes';}
function numericValue(input){return input.value===''?null:input.valueAsNumber;}
function fieldMarkup(side,key,label){return `<fieldset class="steering-point"><legend>${label}</legend><div class="steering-xyz">${['X','Y','Z'].map((axis,index)=>`<label class="field">${axis}<input type="number" step="any" placeholder="mm" data-side="${side}" data-point="${key}" data-axis="${index}" aria-label="${side} ${label} ${axis} in mm"></label>`).join('')}</div></fieldset>`;}
function buildHardpoints(){
  $('steering-hardpoints').innerHTML=sides.map(side=>`<details class="steering-point-side" data-side="${side}" ${side==='left'?'open':''}><summary><span class="steering-side-label"><i class="steering-side-dot"></i>${side==='left'?'Left':'Right'} upright</span></summary><div class="steering-point-fields">${Object.entries(pointNames).map(([key,label])=>fieldMarkup(side,key,label)).join('')}<div class="steering-wheel-fields">${Object.entries(scalarNames).map(([key,label])=>`<label class="field">${label}<input type="number" step="any" placeholder="${key==='wheelCenterZ'?'mm':'deg'}" data-side="${side}" data-scalar="${key}" aria-label="${side} ${label} in ${key==='wheelCenterZ'?'mm':'degrees'}"><small>${key==='wheelCenterZ'?'X = 0; Y = ± front track / 2':key==='staticToeDeg'?'Positive inward · degrees':'Positive top-outward · degrees'}</small></label>`).join('')}</div></div></details>`).join('');
}
function populate(){
  $('steering-name').value=design.name||'';
  for(const input of document.querySelectorAll('[data-param]'))input.value=design[input.dataset.param]??'';
  for(const input of document.querySelectorAll('[data-point]'))input.value=design[input.dataset.side][input.dataset.point][+input.dataset.axis]??'';
  for(const input of document.querySelectorAll('[data-scalar]'))input.value=design[input.dataset.side][input.dataset.scalar]??'';
  angle=0;$('steering-angle').value=0;
  resetRackOffsetDraft();
}
function editorErrors(){
  const errors=[];
  for(const input of document.querySelectorAll('.steering-workspace input[type="number"]:not(#rack-offset)')) {
    const invalid=!input.checkValidity();input.setAttribute('aria-invalid',String(invalid));
    if(invalid)errors.push(`Check ${input.getAttribute('aria-label')}.`);
  }
  return errors;
}
function syncRackOffset(){
  const input=$('rack-offset'),applied=rackLongitudinalOffset(design);
  if(!rackOffsetPending)input.value=applied??'';
  const bothKnown=sides.every(side=>Number.isFinite(design[side]?.tieInner?.[0]));
  const mode=bothKnown?'translate':'set-both',target=input.valueAsNumber;
  let candidate={ok:false,reason:'Enter a finite target X in millimeters, or reset the entry.'};
  if(input.value!==''&&Number.isFinite(target)&&Math.abs(target)<=1e7&&input.checkValidity()){
    try{candidate=setRackLongitudinalOffset(design,target,{mode});}catch(error){candidate={ok:false,reason:`The rack target could not be applied: ${error.message}`};}
  }else if(input.value!==''&&Number.isFinite(target)&&Math.abs(target)>1e7){
    candidate={ok:false,reason:'The rack target must be within ±10000000 mm, the coordinate edit bound.'};
  }
  const invalid=rackOffsetPending&&!candidate.ok;
  input.setAttribute('aria-invalid',String(invalid));
  $('apply-rack-offset').textContent=bothKnown?'Apply rack offset':'Set both rack X values';
  $('apply-rack-offset').disabled=!rackOffsetPending||!candidate.ok;
  $('reset-rack-offset').disabled=!rackOffsetPending;
  const current=Number.isFinite(applied)?`${signed(applied)} mm`:'unknown (both rack-joint X coordinates are not fully defined)';
  const operation=bothKnown?'Applying shifts both rack-joint X coordinates equally, preserving their X difference and all Y/Z values.':'Set both rack X values assigns the target to both inner joints, including any existing X value; their Y/Z values stay unchanged.';
  $('rack-offset-status').textContent=(rackOffsetPending?`Target not applied; calculations use the current rack offset ${current}. `:`Current rack offset: ${current}. `)+(invalid?`${candidate.reason} `:'')+operation;
  $('rack-offset-status').classList.toggle('error',invalid);
  document.querySelector('.steering-rack-offset').classList.toggle('pending',rackOffsetPending);
  return candidate;
}
function resetRackOffsetDraft(){rackOffsetPending=false;syncRackOffset();}
function syncPersistenceControls(inputErrors=editorErrors()){
  let draftErrors;
  try{draftErrors=validateSteeringDesign(design,{allowIncomplete:true});}catch{draftErrors=['Malformed draft'];}
  $('save-steering').disabled=rackOffsetPending||!storageReadable||inputErrors.length>0||draftErrors.length>0;
  $('export-json').disabled=rackOffsetPending||inputErrors.length>0||draftErrors.length>0;
  $('export-csv').disabled=rackOffsetPending||!ready||!rows.length;
}
function applyRackOffset(){
  const candidate=syncRackOffset();
  if(!rackOffsetPending||!candidate.ok)return;
  design=candidate.design;
  for(const side of sides)document.querySelector(`[data-side="${side}"][data-point="tieInner"][data-axis="0"]`).value=design[side].tieInner[0]??'';
  resetRackOffsetDraft();markDirty();update();
  notify(candidate.mode==='set-both'?'Set both rack-joint X coordinates to the entered target. Y/Z values were retained; reference tie-rod lengths are recomputed from the joints.':'Applied the rack midpoint X. Both inner joints moved equally in X; longitudinal staggering and Y/Z were retained. Reference tie-rod lengths are recomputed from the joints.');
}
function renderEngineState(){
  const messages={idle:'Python calculations start when geometry is complete.',loading:'Calculating the steering sweep with Python…',ready:`Python engine connected · ${rows.length} steering positions.`,error:engineError?.message||'The Python engine is unavailable.'};
  $('steering-engine-status').textContent=messages[engineState];
  $('steering-engine').dataset.state=engineState;
  $('retry-steering').hidden=engineState!=='error';$('retry-steering').disabled=!eligible||engineState==='loading';
  if(engineState==='error')$('steering-engine-help').open=true;
  const empty=$('steering-preview-empty');empty.hidden=ready;
  empty.querySelector('strong').textContent=engineState==='loading'?'Calculating steering…':engineState==='error'?'Calculated geometry unavailable.':'Start with your geometry.';
  empty.querySelector('span').textContent=engineState==='loading'?'Previous calculated results are cleared while the Python engine evaluates this layout.':engineState==='error'?'Your draft is retained. Start or reconnect the Python application server, then retry.':'Unknown values stay blank. Enter the dimensions and joint centers, or inspect the labeled synthetic example.';
}
function receiveSweepState(state){
  engineState=state.status;engineError=state.error||null;rows=state.status==='ready'?state.rows:[];ready=state.status==='ready';
  $('steering-angle').disabled=!ready;$('center-steering').disabled=!ready;
  if(engineState==='loading'){$('steering-status').classList.remove('error');$('steering-status').textContent='Waiting for the Python steering sweep. Previous calculated positions have been cleared.';}
  if(engineState==='error'){$('steering-status').classList.add('error');$('steering-status').textContent=engineError?.message||'The Python engine could not calculate this geometry. No calculated results are retained.';}
  renderEngineState();renderTable();showPosition();syncPersistenceControls();
}
function renderReadouts(result){
  const m=result?.ok?result.metrics:null;
  const values=[['Left heading',result?.ok?result.left.headingDeg:null,'°'],['Right heading',result?.ok?result.right.headingDeg:null,'°'],['Rack displacement',result?.ok?result.rack:null,'mm'],['Total toe-in',m?.totalToeInDeg,'°'],['Δ total toe-in',m?.deltaTotalToeInDeg,'°'],['Outside angle error',m?.ackermannErrorDeg,'°'],['Left camber',result?.ok?result.left.camberDeg:null,'°'],['Right camber',result?.ok?result.right.camberDeg:null,'°']];
  $('steering-readouts').innerHTML=values.map(([label,value,unit])=>`<dl class="steering-readout"><dt>${label}</dt><dd>${signed(value)} <small>${unit}</small></dd></dl>`).join('');
  $('turn-badge').textContent=m?m.turn==='straight'?'Straight ahead':m.turn==='mixed'?'Mixed headings':`${m.turn==='right'?'Right':'Left'} turn · ${m.insideSide} inside`:'Awaiting geometry';
}
function syncIntersectionFit(){
  const available=ready&&viewer.showArmLines&&currentConstruction?.intersection?.status==='finite';
  $('fit-arm-intersection').disabled=!available;
  if(!available)viewer.fitIntersection=false;
  $('fit-arm-intersection').setAttribute('aria-pressed',String(viewer.fitIntersection));
  $('fit-arm-intersection').textContent=viewer.fitIntersection?'Reset intersection fit':'Fit intersection';
}
function renderConstruction(result){
  currentConstruction=ready?result?.construction:null;
  const construction=currentConstruction,intersection=construction?.intersection;
  let intersectionMarkup=`<p class="steering-construction-unavailable">${eligible?'Waiting for a valid Python construction result.':'Complete the geometry to evaluate arm lines.'}</p>`;
  if(intersection?.status==='finite'&&intersection.point){
    const [x,y]=intersection.point,offset=intersection.rearAxleOffsetMm;
    const offsetText=Number.isFinite(offset)?Math.abs(offset)<.005?'0.00 mm from rear axle at shown precision':`${signed(offset)} mm · ${offset>0?'ahead of':'behind'} rear axle`:'Rear-axle offset unavailable';
    intersectionMarkup=`<dl class="steering-intersection-coordinates"><div><dt>X</dt><dd>${signed(Math.abs(x)<.005?0:x)} <small>mm</small></dd></div><div><dt>Y</dt><dd>${signed(Math.abs(y)<.005?0:y)} <small>mm</small></dd></div></dl><p class="steering-intersection-offset">${escape(offsetText)}</p>`;
  }else if(intersection){
    const descriptions={parallel:'Parallel arm lines · no finite intersection.',coincident:'Coincident arm lines · no unique intersection.',undefined:'Arm-line intersection unavailable for this geometry.'};
    intersectionMarkup=`<p class="steering-construction-unavailable">${descriptions[intersection.status]||descriptions.undefined}</p>`;
  }
  const state=!ready?(eligible?engineState==='loading'?'Waiting for Python sweep':'Python construction unavailable':'Awaiting complete geometry'):construction?.pose==='solved'?'Selected solved position':'Static reference only · selected position unavailable';
  $('steering-construction').innerHTML=`<div class="steering-construction-heading"><h3>Steering-arm construction</h3><span class="steering-construction-state${ready&&!result?.ok?' unavailable':''}">${state}</span></div><div class="steering-construction-grid">${sides.map(side=>`<section class="steering-construction-card ${side}"><h4>${side==='left'?'Left':'Right'} rod / arm</h4><dl class="steering-construction-angles"><div><dt>α plan</dt><dd>${format(construction?.[side]?.planAngleDeg)}<small>°</small></dd></div><div><dt>β effective 3D</dt><dd>${format(construction?.[side]?.spatialAngleDeg)}<small>°</small></dd></div></dl></section>`).join('')}<section class="steering-construction-card intersection"><h4>Arm-line intersection · front datum</h4>${intersectionMarkup}</section></div><details class="steering-construction-definitions"><summary>Angle definitions & intersection meaning</summary><p>Both included angles are measured at the outer tie-rod joint toward the inner rod joint. For α, the other ray points to steering-axis point K at the outer joint’s height, and both rays are projected into the X–Y plane. For β, the other ray points to the perpendicular foot C on the steering axis, using the full 3D vectors. These are different arm references, not simply two views of one angle.</p><p>The plan arm extensions pass through K and the outer joint. Their intersection is a geometric construction point, not the vehicle’s rolling turn center. Rear-axle offset is X<sub>intersection</sub> + wheelbase: positive ahead of the rear axle, negative behind. Coordinates and offset use millimeters in the front-axle datum. Undefined values remain blank.</p></details>`;
  syncIntersectionFit();
}
function renderTable(){
  if(!rows.length){const message=engineState==='loading'?'Waiting for the Python sweep. Previous results have been cleared.':engineState==='error'?'No calculated sweep. Reconnect the Python engine and retry.':'No calculated sweep. Complete the required geometry and correct any invalid values.';$('steering-sweep').innerHTML=`<tr><td colspan="12" class="steering-no-data">${message}</td></tr>`;$('sweep-summary').textContent=engineState==='loading'?'Calculating every 5° steering-wheel increment with Python.':engineState==='error'?'The Python engine has not returned a valid sweep.':'Enter complete geometry to calculate every 5° steering-wheel increment.';return;}
  const failures=rows.filter(row=>!row.ok).length;
  $('sweep-summary').textContent=`${rows.length} positions · ${-design.steeringLimitDeg}° to +${design.steeringLimitDeg}° · 5° steering-wheel steps${failures?` · ${failures} unavailable position${failures===1?'':'s'}`:''}. Select a row to inspect it.`;
  $('steering-sweep').innerHTML=rows.map(row=>{
    const input=row.steeringWheelDeg,selected=input===angle?' selected':'';
    if(!row.ok)return `<tr class="failed${selected}" data-angle="${input}" tabindex="0" aria-label="Inspect unavailable ${input} degree input"><td>${signed(input,0)}°</td><td>${format(row.rack)}</td><td colspan="10" class="steering-no-data">${escape(row.reason||'No assembly solution.')}</td></tr>`;
    const m=row.metrics,construction=row.construction;
    return `<tr class="${selected.trim()}" data-angle="${input}" tabindex="0" aria-label="Inspect ${input} degree steering wheel input"><td>${signed(input,0)}°</td><td>${signed(row.rack)}</td><td>${signed(row.left.headingDeg)}</td><td>${signed(row.right.headingDeg)}</td><td>${format(construction.left.planAngleDeg)} / ${format(construction.left.spatialAngleDeg)}</td><td>${format(construction.right.planAngleDeg)} / ${format(construction.right.spatialAngleDeg)}</td><td>${format(m.insideDeg)} / ${format(m.outsideDeg)}</td><td>${signed(m.totalToeInDeg)}</td><td>${signed(m.deltaTotalToeInDeg)}</td><td>${format(m.idealOutsideDeg)}</td><td>${signed(m.ackermannErrorDeg)}</td><td>${format(m.ackermannPercent,1)}</td></tr>`;
  }).join('');
}
function showPosition(){
  $('steering-angle').value=angle;$('steering-angle-value').textContent=`${signed(angle,0)}°`;
  if(!ready){renderReadouts(null);renderConstruction(null);viewer.update(design,null,false);chart.update([],angle);return;}
  const result=rows.find(row=>row.steeringWheelDeg===angle);
  if(!result){receiveSweepState({status:'error',error:new Error('The selected position is missing from the Python sweep. Retry the calculation.')});return;}
  renderConstruction(result);viewer.update(design,result,true);chart.update(rows,angle);renderReadouts(result);
  $('steering-status').classList.toggle('error',!result.ok);
  if(!result.ok)$('steering-status').textContent=`This position is unavailable: ${result.reason||'The linkage did not produce a valid assembly solution.'} The preview shows entered static geometry only.`;
  else {
    const residual=Math.max(Math.abs(result.left.residual),Math.abs(result.right.residual));
    const m=result.metrics,comparison=Number.isFinite(m.idealOutsideDeg)?`Inside ${format(m.insideDeg)}° · outside ${format(m.outsideDeg)}° · nominal ideal outside ${format(m.idealOutsideDeg)}°.`:'Nominal Ackermann comparison is undefined at this position.';
    $('steering-status').textContent=`Rigid-link position solved by Python. ${comparison} Maximum tie-rod length residual ${Number.isFinite(residual)?residual.toExponential(2):'unavailable'} mm. Numerical closure is not physical validation.`;
  }
  for(const row of $('steering-sweep').querySelectorAll('[data-angle]'))row.classList.toggle('selected',+row.dataset.angle===angle);
}
function update(){
  const inputErrors=editorErrors();let errors;
  try{errors=[...inputErrors,...validateSteeringDesign(design)];}catch(error){errors=[`Invalid geometry data: ${error.message}`];}
  eligible=errors.length===0;
  if(eligible){
    const limit=design.steeringLimitDeg;$('steering-angle').min=-limit;$('steering-angle').max=limit;
    angle=Math.max(-limit,Math.min(limit,angle));$('steering-min-label').textContent=`−${limit}°`;$('steering-max-label').textContent=`+${limit}°`;
    void sweepLoader.load(design);
  }else{
    sweepLoader.cancel();$('steering-min-label').textContent='− limit';$('steering-max-label').textContent='+ limit';
    const missing=errors.length;
    $('steering-status').classList.toggle('error',inputErrors.length>0);
    $('steering-status').innerHTML=`${missing} input ${missing===1?'item needs':'items need'} attention before calculating. Unknown values are never replaced with zero.<details><summary>Review required inputs</summary><ul>${errors.map(error=>`<li>${escape(error)}</li>`).join('')}</ul></details>`;
  }
  $('export-ready').hidden=true;
  syncRackOffset();syncPersistenceControls(inputErrors);
}
function download(content,type,filename){
  if(exportUrl)URL.revokeObjectURL(exportUrl);
  exportUrl=URL.createObjectURL(new Blob([content],{type}));
  const anchor=$('export-download');anchor.href=exportUrl;anchor.download=filename;
  $('export-ready-status').textContent=filename;
  $('export-contents').value=content;$('export-contents-details').open=false;
  $('export-ready').hidden=false;
  notify(`Export prepared: ${filename}. Use Download file, or view and copy its contents if browser downloads are unavailable.`);
  $('export-ready').scrollIntoView({behavior:'smooth',block:'nearest'});
  anchor.click();
}
function basename(){return (design.name||'steering-design').replace(/[^a-z0-9_-]+/gi,'-').replace(/^-|-$/g,'').slice(0,70)||'steering-design';}
function exportCsv(){
  const headings=['steering_wheel_deg','rack_mm','status','reason','left_heading_deg','right_heading_deg','left_rod_arm_plan_angle_deg','left_rod_arm_effective_3d_angle_deg','right_rod_arm_plan_angle_deg','right_rod_arm_effective_3d_angle_deg','left_toe_in_deg','right_toe_in_deg','left_delta_toe_in_deg','right_delta_toe_in_deg','inside_side','inside_deg','outside_deg','total_toe_in_deg','delta_total_toe_in_deg','nominal_ideal_outside_deg','outside_error_deg','ackermann_percent','cot_residual','left_camber_deg','right_camber_deg','left_tie_residual_mm','right_tie_residual_mm','arm_line_intersection_status','arm_line_intersection_x_mm','arm_line_intersection_y_mm','arm_line_intersection_rear_offset_mm'];
  const data=rows.map(r=>{const m=r.metrics||{},l=r.left||{},right=r.right||{},construction=r.ok?r.construction:null,intersection=construction?.intersection;return[r.steeringWheelDeg,r.rack,r.ok?'solved':'unavailable',r.reason||'',l.headingDeg,right.headingDeg,construction?.left.planAngleDeg,construction?.left.spatialAngleDeg,construction?.right.planAngleDeg,construction?.right.spatialAngleDeg,l.toeInDeg,right.toeInDeg,l.deltaToeInDeg,right.deltaToeInDeg,m.insideSide,m.insideDeg,m.outsideDeg,m.totalToeInDeg,m.deltaTotalToeInDeg,m.idealOutsideDeg,m.ackermannErrorDeg,m.ackermannPercent,m.cotResidual,l.camberDeg,right.camberDeg,l.residual,right.residual,intersection?.status,intersection?.point?.[0],intersection?.point?.[1],intersection?.rearAxleOffsetMm];});
  const cell=value=>value==null||typeof value==='number'&&!Number.isFinite(value)?'':`"${String(value).replaceAll('"','""')}"`;
  download([headings,...data].map(row=>row.map(cell).join(',')).join('\r\n'),'text/csv;charset=utf-8',`${basename()}-sweep.csv`);
}

buildHardpoints();
try {
  const saved=localStorage.getItem(STORAGE);
  if(saved){const parsed=JSON.parse(saved),errors=validateSteeringDesign(parsed,{allowIncomplete:true});if(errors.length){storageReadable=false;notify('The saved steering draft could not be opened. It has been retained and Save is disabled. Export JSON to keep new work.',true);}else{design=parsed;$('steering-save-state').textContent='Saved draft restored';notify('Restored the steering draft saved in this browser.');}}
}catch{storageReadable=false;notify('Browser storage could not be read. Export JSON to keep your work; the stored draft will not be overwritten.',true);}
syncViewDescription();populate();update();

$('steering-workspace').addEventListener('input',event=>{
  const input=event.target;
  if(input.dataset.param)design[input.dataset.param]=numericValue(input);
  else if(input.dataset.point)design[input.dataset.side][input.dataset.point][+input.dataset.axis]=numericValue(input);
  else if(input.dataset.scalar)design[input.dataset.side][input.dataset.scalar]=numericValue(input);
  else if(input.id==='steering-name')design.name=input.value;
  else return;
  markDirty();update();
});
$('rack-offset').addEventListener('input',()=>{rackOffsetPending=true;syncRackOffset();syncPersistenceControls();$('export-ready').hidden=true;});
$('rack-offset').addEventListener('keydown',event=>{if(event.key==='Enter'){event.preventDefault();applyRackOffset();}});
$('apply-rack-offset').addEventListener('click',applyRackOffset);
$('reset-rack-offset').addEventListener('click',()=>{resetRackOffsetDraft();syncPersistenceControls();});
$('steering-angle').addEventListener('input',event=>{angle=+event.target.value;showPosition();});
$('retry-steering').addEventListener('click',()=>{if(eligible)update();});
$('center-steering').addEventListener('click',()=>{angle=0;showPosition();});
$('ideal-overlay').addEventListener('change',event=>{viewer.showIdeal=event.target.checked;viewer.draw();});
$('steering-mode-3d').addEventListener('click',()=>setViewMode('3d'));
$('steering-mode-plan').addEventListener('click',()=>setViewMode('plan'));
$('steering-camera').addEventListener('change',event=>{if(spatialViewer&&event.target.value!=='orbit')spatialViewer.setView(event.target.value);});
$('fit-steering-view').addEventListener('click',()=>spatialViewer?.fit());
$('steering-view').addEventListener('click',()=>{viewer.frontOnly=!viewer.frontOnly;viewer.fitIntersection=false;syncIntersectionFit();$('steering-view').setAttribute('aria-pressed',String(viewer.frontOnly));$('steering-view').textContent=viewer.frontOnly?'Show full vehicle':'Focus front linkage';viewer.draw();});
$('arm-lines-overlay').addEventListener('change',event=>{viewer.showArmLines=event.target.checked;syncIntersectionFit();viewer.draw();});
$('arm-angles-overlay').addEventListener('change',event=>{viewer.showArmAngles=event.target.checked;viewer.draw();});
$('fit-arm-intersection').addEventListener('click',()=>{if($('fit-arm-intersection').disabled)return;viewer.fitIntersection=!viewer.fitIntersection;syncIntersectionFit();viewer.draw();});
$('steering-chart-mode').addEventListener('change',event=>{
  chart.mode=event.target.value;
  const descriptions={headings:'Signed heading versus steering-wheel input · degrees',toe:'Change from static total toe-in versus steering-wheel input · degrees',error:'Actual outside minus nominal ideal outside angle · degrees'};
  $('chart-description').textContent=descriptions[chart.mode];$('steering-chart').setAttribute('aria-label',descriptions[chart.mode]);
  $('chart-legend').innerHTML=(chart.mode==='headings'?'<span><i class="legend-left"></i> Left wheel</span><span><i class="legend-right"></i> Right wheel</span>':chart.mode==='toe'?'<span><i class="legend-left"></i> Δ total toe-in</span>':'<span><i class="legend-ideal"></i> Outside angle error</span>')+'<span><i class="legend-cursor"></i> Selected input</span>';
  chart.draw();
});
$('mirror-steering').addEventListener('click',()=>{
  for(const key of Object.keys(pointNames))design.right[key]=design.left[key].map((value,index)=>value==null?null:index===1?-value:value);
  for(const key of Object.keys(scalarNames))design.right[key]=design.left[key];
  populate();markDirty();update();notify('Mirrored left joint coordinates across Y = 0 and copied the side-defined static toe and camber. Both sides remain independently editable.');
});
$('new-steering').addEventListener('click',()=>{design=newSteeringDesign();populate();markDirty();update();notify('New blank steering design. Required geometry remains unknown until entered.');});
$('load-example').addEventListener('click',()=>{design=exampleSteeringDesign();populate();markDirty();update();notify('Synthetic example loaded. These coordinates are for software demonstration; they are not measured or recommended vehicle geometry.');});
$('save-steering').addEventListener('click',()=>{try{localStorage.setItem(STORAGE,JSON.stringify(design));dirty=false;$('steering-save-state').textContent='Saved in this browser';notify('Steering draft saved in this browser. Export JSON to keep a portable copy.');}catch{notify('The draft could not be saved in browser storage. Use Export JSON to keep it.',true);}});
$('export-json').addEventListener('click',()=>download(JSON.stringify(design,null,2),'application/json',`${basename()}.json`));
$('export-csv').addEventListener('click',exportCsv);
$('close-export').addEventListener('click',()=>{$('export-ready').hidden=true;});
$('import-steering').addEventListener('change',async event=>{
  const file=event.target.files?.[0];if(!file)return;
  try{const parsed=JSON.parse(await file.text()),errors=validateSteeringDesign(parsed,{allowIncomplete:true});if(errors.length)throw new Error(errors.join(' '));design=parsed;populate();markDirty();update();notify(`Imported ${file.name}. Review all dimensions, coordinate signs and model assumptions before interpreting results.`);}catch(error){notify(`Import failed. Current draft retained. ${error.message}`,true);}finally{event.target.value='';}
});
function selectRow(event){const row=event.target.closest('[data-angle]');if(!row||!ready)return;angle=+row.dataset.angle;showPosition();}
$('steering-sweep').addEventListener('click',selectRow);
$('steering-sweep').addEventListener('keydown',event=>{if(event.key==='Enter'||event.key===' '){event.preventDefault();selectRow(event);}});
window.addEventListener('beforeunload',event=>{if(dirty||rackOffsetPending){event.preventDefault();event.returnValue='';}});
