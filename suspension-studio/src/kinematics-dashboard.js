import {newKinematicsSettings, completeKinematicsSettings, validateKinematicsSettings} from './kinematics-settings.js?v=0.3.0';
import {createKinematicsSweepLoader} from './kinematics-api.js?v=0.3.0';
import {renderKinematicsChart, exportKinematicsCsv, formatKinematicsValue} from './kinematics-charts.js?v=0.3.0';

const escape = value => String(value ?? '').replace(/[&<>"']/g, char => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[char]));
const finite = value => typeof value === 'number' && Number.isFinite(value);
const MODES = {
  heave:{label:'Heave',unit:'mm',limit:100,description:'Equal wheel-center travel at all four corners. Positive travel moves the wheels upward relative to the fixed chassis.'},
  pitch:{label:'Pitch',unit:'deg',limit:10,description:'Prescribed wheel travel varies with longitudinal distance from the entered rig reference X. Positive input commands front rebound and rear bump about that reference (body-equivalent nose-up). The chassis stays fixed.'},
  roll:{label:'Roll',unit:'deg',limit:10,description:'Prescribed wheel travel varies with lateral distance from the entered rig reference Y. Positive input commands right bump and left rebound about that reference (body-equivalent right-side-down). The chassis stays fixed.'},
  steering:{label:'Steering',unit:'mm',limit:150,description:'Both rack ends translate together in +Y. Wheel-center bump remains zero; wheel orientation follows the solved upright.'},
};
const COLORS = {front:'var(--kin-front-left, #397ba6)',rear:'var(--kin-rear-left, #8f68aa)',vehicle:'var(--kin-vehicle, #6e7884)', 'front.left':'var(--kin-front-left, #397ba6)','front.right':'var(--kin-front-right, #c77e45)','rear.left':'var(--kin-rear-left, #8f68aa)','rear.right':'var(--kin-rear-right, #5e997c)'};
const GROUPS = ['Alignment','Actuation','Centers','Steering','Dimensions','Diagnostics','Hardpoints','Anti-effects','Anti-roll bar'];
const OVERVIEW = new Set(['camber','toe','motionRatio','compression','rcHeight','rcY','camberGain','toeGain','steer','ackermannPercent','caster','kpi','trail','scrub','rollAxis','brakeAnti','driveAnti','arbTwist']);
const initialPlots = [
  {id:'camber',title:'Camber angle',unit:'deg'}, {id:'toe',title:'Toe-in angle',unit:'deg'},
  {id:'motion-ratio',title:'Motion ratio · damper / wheel',unit:'mm/mm'}, {id:'compression',title:'Damper compression',unit:'mm'},
  {id:'roll-center-height',title:'Roll-center height',unit:'mm'}, {id:'roll-center-lateral',title:'Roll-center lateral migration',unit:'mm'},
];
const input = (id, label, {unit='',min,max,step='any',setting='',placeholder='Unknown'}={}) => `<label class="kinematics-field" for="${id}"><span>${label}${unit?` <small>${unit}</small>`:''}</span><input id="${id}" type="number" data-kinematics-input="true" ${setting?`data-kin-setting="${setting}"`:''} step="${step}"${min!==undefined?` min="${min}"`:''}${max!==undefined?` max="${max}"`:''} placeholder="${placeholder}" aria-label="${label}${unit?` in ${unit}`:''}"></label>`;

/** Mount once. The preview host remains stable so the existing canvas can move here. */
export function createKinematicsDashboard(container, {getDesign,onSettingsChange=()=>{},onPoseChange=()=>{},onAxleChange=()=>{},onInputsEdited=()=>{}}) {
  if (!container || typeof getDesign !== 'function') throw new TypeError('A dashboard container and design getter are required.');
  container.classList.add('kinematics-dashboard');
  const initial = getDesign();
  const ranges = {
    heave:{min:-(initial?.heaveTravel?.rebound ?? 25.4),max:initial?.heaveTravel?.compression ?? 25.4,samples:41},
    pitch:{min:-1.5,max:1.5,samples:41},roll:{min:-1.5,max:1.5,samples:41},steering:{min:-15,max:15,samples:41},
  };
  let mode='heave',activeAxle='front',validDesign=false,fingerprint='',data=null,selectedIndex=0,status='idle',message='',lastRequest=null;
  const inputDrafts=new Map();
  const draftKey=key=>`${key.startsWith('reference.')||key.startsWith('forceReference.')?'vehicle':activeAxle}:${key}`;
  let filterGroup='overview',filterAxle='all',filterSide='all',xChannel='input';
  container.innerHTML=`
    <header class="kinematics-intro"><div><p class="kinematics-eyebrow">SUSPENSION ANALYSIS</p><h2>Explore kinematics</h2><p>Follow the geometry through travel. Compare four corners, inspect a pose, and export the calculated channels.</p></div><span class="kinematics-engine-badge" data-kin-status="idle">Python engine</span></header>
    <div class="kinematics-setup-grid">
      <section class="kinematics-controls" aria-labelledby="kinematics-motion-title"><div class="kinematics-card-title"><span class="kinematics-step">01</span><div><h3 id="kinematics-motion-title">Define a sweep</h3><p>One input, both axles.</p></div></div>
        <div class="kinematics-modes" role="group" aria-label="Kinematic motion mode">${Object.entries(MODES).map(([key,item])=>`<button type="button" data-kin-mode="${key}" aria-pressed="${key===mode}">${item.label}</button>`).join('')}</div>
        <p id="kinematics-mode-description" class="kinematics-help"></p>
        <div class="kinematics-range-fields">${input('kinematics-min','From',{unit:'mm',min:-100,max:100,placeholder:'Minimum'})}${input('kinematics-max','To',{unit:'mm',min:-100,max:100,placeholder:'Maximum'})}${input('kinematics-samples','Samples',{min:3,max:81,step:1,placeholder:'3–81'})}</div>
        <div id="kinematics-reference" class="kinematics-reference"><div class="kinematics-subheading"><h4>Rig reference</h4><span>Front-origin coordinates</span></div><div class="kinematics-reference-fields">${input('kinematics-reference-x','Reference X',{unit:'mm',min:-20000,max:20000,setting:'reference.x'})}${input('kinematics-reference-y','Reference Y',{unit:'mm',min:-20000,max:20000,setting:'reference.y'})}</div><div class="kinematics-inline-actions"><button type="button" data-kin-action="front-datum">Use front datum [0, 0]</button><button type="button" data-kin-action="middle-datum">Use mid-wheelbase datum</button></div><p class="kinematics-help">Pitch needs X; roll needs Y. This is a prescribed travel reference, not an inferred CG or roll center.</p></div>
        <div class="kinematics-run-row"><button type="button" id="kinematics-run" class="kinematics-primary">Run sweep <span aria-hidden="true">→</span></button><button type="button" id="kinematics-cancel" hidden>Cancel</button></div>
        <p id="kinematics-readiness" class="kinematics-help"></p>
      </section>
      <section class="kinematics-inputs" aria-labelledby="kinematics-geometry-title"><div class="kinematics-card-title"><span class="kinematics-step">02</span><div><h3 id="kinematics-geometry-title">Complete the model inputs</h3><p>Unknown measurements remain blank.</p></div><label class="kinematics-axle-label"><span class="kinematics-sr-only">Model input axle</span><select id="kinematics-input-axle" data-kinematics-input="true"><option value="front">Front axle</option><option value="rear">Rear axle</option></select></label></div>
        <div class="kinematics-alignment-fields">${input('kinematics-camber','Static camber',{unit:'deg',min:-45,max:45,setting:'staticCamberDeg'})}${input('kinematics-toe','Static toe-in',{unit:'deg',min:-45,max:45,setting:'staticToeInDeg'})}</div>
        <div class="kinematics-inline-actions"><button type="button" data-kin-action="zero-alignment">Set both alignment values to 0°</button></div><p class="kinematics-help">Camber: top outward is positive. Toe-in: inward is positive. These values describe the entered static assembly and mirror across the axle.</p>
        <div class="kinematics-subheading"><h4>Rocker axis direction</h4><span>Through the entered rocker pivot</span></div><div class="kinematics-axis-fields">${['X','Y','Z'].map((axis,index)=>input(`kinematics-axis-${index}`,axis,{min:-10000,max:10000,setting:`rockerAxis.${index}`})).join('')}</div>
        <div class="kinematics-inline-actions"><button type="button" data-kin-action="longitudinal-axis">Set longitudinal axis [1, 0, 0]</button></div><p id="kinematics-axis-help" class="kinematics-help"></p>
        <details class="kinematics-extra-inputs"><summary>Anti-effects & U-bar model inputs</summary>
          <h4>Vehicle-wide nominal force reference</h4><div class="kinematics-reference-fields">${input('kinematics-cg-height','CG height above static ground',{unit:'mm',min:0.001,max:10000,setting:'forceReference.cgHeightMm'})}${input('kinematics-brake-share','Front braking force fraction',{min:0,max:1,setting:'forceReference.frontBrakeFraction'})}${input('kinematics-drive-share','Front drive force fraction',{min:0,max:1,setting:'forceReference.frontDriveFraction'})}</div>
          <p class="kinematics-help">Fractions range from 0 to 1; rear uses the complement. The nominal wheelbase/CG-height reference stays fixed through the rig sweep. Equal left/right longitudinal force split; no predicted body pitch.</p>
          <h4>Selected axle · spin-torque reaction</h4><div class="kinematics-reference-fields"><label class="kinematics-field">Braking<select data-kin-choice="brakeReaction" data-kinematics-input="true" aria-label="Brake spin-torque reaction"><option value="">Unknown</option><option value="upright">Upright / outboard brake</option><option value="chassis">Chassis / inboard brake</option></select></label><label class="kinematics-field">Drive<select data-kin-choice="driveReaction" data-kinematics-input="true" aria-label="Drive spin-torque reaction"><option value="">Unknown</option><option value="chassis">Chassis</option><option value="upright">Upright</option></select></label></div>
          <h4>Selected axle · anti-roll bar</h4><label class="kinematics-field">ARB kinematic model<select data-kin-choice="arbModel" data-kinematics-input="true" aria-label="ARB kinematic model"><option value="">Not selected · static preview only</option><option value="straight-transverse">Straight shaft through mirrored bar bends</option></select></label><label class="kinematics-field">Suspension pickup carried by<select data-kin-choice="arbAttachment" data-kinematics-input="true" aria-label="ARB suspension pickup carrier"><option value="">Unknown</option><option value="upright">Upright</option><option value="lower-wishbone">Lower wishbone</option><option value="upper-wishbone">Upper wishbone</option><option value="rocker">Actuation rocker</option></select></label>
          <p class="kinematics-help">Enable the U-bar and enter its three geometry points in Configure vehicle. This model uses rigid lever arms, spherical-ended drop links and a straight transverse shaft. An entered bearing must lie on the shaft. Arm rotation and relative twist are calculated; stiffness and forces are not inferred.</p>
        </details><div class="kinematics-model-note"><strong>Rigid geometry model</strong><span>Fixed chassis · prescribed wheel travel · ideal circular tire reference. Alignment is chassis-relative. Motion ratio is damper compression / upward wheel travel.</span></div>
      </section>
    </div>
    <div id="kinematics-status" class="kinematics-status" role="status" aria-live="polite"></div>
    <section class="kinematics-inspection" aria-label="Selected suspension position"><div class="kinematics-selection"><div><p class="kinematics-eyebrow">SELECTED POSITION</p><h3 id="kinematics-position-title">No sweep calculated</h3></div><div class="kinematics-position-controls"><button type="button" id="kinematics-previous" aria-label="Previous calculated sample">‹</button><label for="kinematics-position" class="kinematics-sr-only">Calculated sample index</label><input type="range" id="kinematics-position" data-kinematics-input="true" min="0" max="0" step="1" value="0" disabled><button type="button" id="kinematics-next" aria-label="Next calculated sample">›</button></div><span id="kinematics-position-count">—</span></div><div id="kinematics-preview-host"></div><div id="kinematics-selected-summary" class="kinematics-selected-summary"></div><details id="kinematics-unavailable-details" class="kinematics-unavailable" hidden><summary id="kinematics-unavailable-title"></summary><ul id="kinematics-unavailable-list"></ul></details></section>
    <section class="kinematics-results" aria-labelledby="kinematics-results-title"><div class="kinematics-results-heading"><div><p class="kinematics-eyebrow">PERFORMANCE CHANNELS</p><h3 id="kinematics-results-title">Kinematic performance</h3><p id="kinematics-chart-count">Solved samples will appear here.</p></div><button type="button" id="kinematics-export" disabled>Export results CSV <span aria-hidden="true">↗</span></button></div>
      <div class="kinematics-filters"><label>Show<select id="kinematics-group" data-kinematics-input="true"><option value="overview">Performance dashboard</option><option value="all">All graph families</option>${GROUPS.map(group=>`<option value="${group}">${group}</option>`).join('')}</select></label><label>Axles<select id="kinematics-filter-axle" data-kinematics-input="true"><option value="all">Both axles</option><option value="front">Front</option><option value="rear">Rear</option></select></label><label>Sides<select id="kinematics-filter-side" data-kinematics-input="true"><option value="all">Both sides</option><option value="left">Left</option><option value="right">Right</option></select></label><label class="kinematics-x-axis">Horizontal axis<select id="kinematics-x-axis" data-kinematics-input="true"><option value="input">Sweep input</option></select></label></div>
      <div id="kinematics-chart-grid" class="kinematics-chart-grid"></div>
    </section>
    <details class="kinematics-coverage"><summary><span>Channel coverage, definitions & model limits</span><span aria-hidden="true">+</span></summary><p>Coverage follows documented kinematic channel families. This is not a claim of all-channel or accuracy equivalence with OptimumKinematics. Missing geometry and singular constructions are reported as unavailable.</p><div id="kinematics-coverage-list"></div><div id="kinematics-assumptions"></div><a href="docs/KINEMATICS-CHANNELS.md" target="_blank" rel="noopener">Read the channel definitions and source comparison ↗</a></details>`;
  const $ = id => container.querySelector(`#${id}`);
  const discard=document.createElement('button');discard.type='button';discard.className='button subtle';discard.textContent='Discard unaccepted model inputs';discard.hidden=true;discard.dataset.kinAction='discard-inputs';
  $('kinematics-readiness').after(discard);
  const loader = createKinematicsSweepLoader({onState:receiveState});
  const settings = () => completeKinematicsSettings(getDesign().kinematics);

  function settingFields() {
    const value=settings(),axle=value.axles[activeAxle],design=getDesign();
    for (const element of container.querySelectorAll('[data-kin-setting]')) {
      if (document.activeElement===element) continue;
      const key=element.dataset.kinSetting;
      const draft=inputDrafts.get(draftKey(key));
      if(draft){element.value=draft.value;element.setCustomValidity('Correct or discard this unaccepted model input.');element.setAttribute('aria-invalid','true');continue;}
      const entered=key.startsWith('reference.')?value.reference[key.split('.')[1]]:key.startsWith('forceReference.')?value.forceReference[key.split('.')[1]]:key.startsWith('rockerAxis.')?axle.rockerAxis[Number(key.split('.')[1])]:axle[key];
      element.value=entered ?? '';element.setCustomValidity('');element.removeAttribute('aria-invalid');
    }
    for(const element of container.querySelectorAll('[data-kin-choice]'))element.value=axle[element.dataset.kinChoice]??'';
    $('kinematics-input-axle').value=activeAxle;
    const needsAxis=design.axles[activeAxle].topology!=='macpherson'&&design.axles[activeAxle].actuation!=='direct';
    for (const element of container.querySelectorAll('[data-kin-setting^="rockerAxis."],[data-kin-action="longitudinal-axis"]'))element.disabled=!needsAxis;
    $('kinematics-axis-help').textContent=needsAxis?'Enter any nonzero direction vector for the real bearing axis. Without it, wheel poses can solve but rocker motion and damper channels remain unavailable.':'This axle uses direct actuation or a strut, so a separate rocker axis is not used. Existing values are retained.';
    const middle=container.querySelector('[data-kin-action="middle-datum"]');
    middle.textContent=`Use mid-wheelbase [${formatKinematicsValue(-design.wheelbase/2)}, 0]`;
  }
  function modeFields() {
    const item=MODES[mode],range=ranges[mode];
    container.querySelectorAll('[data-kin-mode]').forEach(button=>button.setAttribute('aria-pressed',String(button.dataset.kinMode===mode)));
    $('kinematics-mode-description').textContent=item.description;
    for(const key of ['min','max']){const element=$(`kinematics-${key}`);element.value=range[key];element.min=-item.limit;element.max=item.limit;element.closest('label').querySelector('small').textContent=item.unit;element.setAttribute('aria-label',`${key==='min'?'From':'To'} in ${item.unit}`);}
    $('kinematics-samples').value=range.samples;
    $('kinematics-reference').hidden=!['pitch','roll'].includes(mode);
    $('kinematics-reference-x').closest('label').classList.toggle('kinematics-field-required',mode==='pitch');
    $('kinematics-reference-y').closest('label').classList.toggle('kinematics-field-required',mode==='roll');
  }
  function readiness() {
    const errors=[];
    discard.hidden=!inputDrafts.size;
    if(inputDrafts.size)errors.push('Correct or discard unaccepted model inputs before saving or solving. Entries are retained across axle switches.');
    if(!validDesign)errors.push('Correct the design or hardpoint inputs before running.');
    for(const element of container.querySelectorAll('input[type="number"]:not(:disabled)')){
      const optional=!!element.dataset.kinSetting;
      const invalid=!element.validity.valid||(!optional&&element.value==='');
      element.setAttribute('aria-invalid',String(invalid));
      if(invalid)errors.push(`Check ${element.getAttribute('aria-label')}.`);
    }
    const range=ranges[mode],current=settings();
    if(!(finite(range.min)&&finite(range.max)&&range.min<range.max))errors.push('The sweep minimum must be smaller than its maximum.');
    if(!Number.isInteger(range.samples)||range.samples<3||range.samples>81)errors.push('Use 3–81 whole-number samples.');
    if(mode==='pitch'&&!finite(current.reference.x))errors.push('Enter a rig reference X for pitch, or select a datum.');
    if(mode==='roll'&&!finite(current.reference.y))errors.push('Enter a rig reference Y for roll, or select a datum.');
    if(mode==='steering')for(const [name,axle] of Object.entries(getDesign().axles))if(axle.steered&&Math.max(Math.abs(range.min),Math.abs(range.max))>axle.rackTravel/2)errors.push(`The ${name} rack allows ±${formatKinematicsValue(axle.rackTravel/2)} mm.`);
    errors.push(...validateKinematicsSettings(current));
    $('kinematics-run').disabled=status==='loading'||errors.length>0;
    $('kinematics-readiness').textContent=errors[0]||'The sweep can run. Blank alignment or rocker-axis inputs leave their dependent channels unavailable.';
    $('kinematics-readiness').classList.toggle('error',errors.length>0);
    return errors;
  }
  function renderStatus() {
    const badge=container.querySelector('.kinematics-engine-badge');
    badge.dataset.kinStatus=status;badge.textContent=status==='ready'?'Python · results ready':status==='loading'?'Python · calculating':status==='error'?'Python · unavailable':'Python engine';
    const text=message||(status==='loading'?'Solving both axles and evaluating kinematic channels…':status==='ready'?`${data.rows.length} sampled positions calculated. Numerical closure checks are shown below; this is not experimental vehicle validation.`:'Choose a motion and run the sweep to calculate the dashboard.');
    $('kinematics-status').textContent=text;$('kinematics-status').dataset.state=status;
    $('kinematics-cancel').hidden=status!=='loading';
    $('kinematics-export').disabled=status!=='ready';readiness();
  }
  function row(){return status==='ready'?data?.rows[selectedIndex]??null:null;}
  function notifyPose(){onPoseChange(row()?.poses??null,row());}
  function selectedView() {
    const selected=row(),count=data?.rows.length??0,position=$('kinematics-position');
    position.disabled=!selected;position.max=Math.max(0,count-1);position.value=selectedIndex;
    position.setAttribute('aria-valuetext',selected?`${MODES[mode].label} ${formatKinematicsValue(selected.input)} ${MODES[mode].unit}`:'No calculated sample');
    $('kinematics-previous').disabled=!selected||selectedIndex===0;$('kinematics-next').disabled=!selected||selectedIndex===count-1;
    $('kinematics-position-title').textContent=selected?`${MODES[mode].label} ${formatKinematicsValue(selected.input)} ${MODES[mode].unit}`:'No sweep calculated';
    $('kinematics-position-count').textContent=selected?`Sample ${selectedIndex+1} / ${count}`:'—';
    if(!selected){$('kinematics-selected-summary').innerHTML='';$('kinematics-unavailable-details').hidden=true;return;}
    const solved=Object.values(selected.poses).reduce((sum,pose)=>sum+Number(pose.left.ok)+Number(pose.right.ok),0);
    const resolved=Object.values(selected.values).filter(finite).length;
    const cells=[['Solved corners',`${solved} / 4`],['Available channels',`${resolved} / ${data.channels.length}`],['Maximum closure residual',`${formatKinematicsValue(selected.diagnostics.maxResidualMm)}${finite(selected.diagnostics.maxResidualMm)?' mm':''}`],['Preview axle',`${activeAxle==='front'?'Front':'Rear'} · chassis-fixed`]];
    $('kinematics-selected-summary').innerHTML=cells.map(([label,value])=>`<div><span>${escape(label)}</span><strong>${escape(value)}</strong></div>`).join('');
    const reasons=new Map();
    Object.values(selected.unavailable).forEach(reason=>reasons.set(reason,(reasons.get(reason)||0)+1));
    for(const [name,pose] of Object.entries(selected.poses))for(const side of ['left','right'])if(!pose[side].ok)reasons.set(`${name} ${side}: ${pose[side].reason}`,0);
    $('kinematics-unavailable-details').hidden=!reasons.size;
    $('kinematics-unavailable-title').textContent=`${data.channels.length-resolved} channels unavailable at this position · view reasons`;
    $('kinematics-unavailable-list').innerHTML=[...reasons].map(([reason,count])=>`<li>${escape(reason)}${count?` <span>(${count} channel${count===1?'':'s'})</span>`:''}</li>`).join('');
  }
  function relevant(channel) {
    return(filterGroup==='all'||filterGroup==='overview'&&OVERVIEW.has(channel.plot)||channel.group===filterGroup)&&(filterAxle==='all'||channel.entity.startsWith('vehicle')||channel.entity.startsWith(filterAxle))&&(filterSide==='all'||!channel.entity.includes('.')||channel.entity.endsWith(`.${filterSide}`));
  }
  function renderCharts() {
    const grid=$('kinematics-chart-grid');
    if(!data||status!=='ready'){
      grid.innerHTML=initialPlots.map(plot=>renderKinematicsChart({...plot,xLabel:`${MODES[mode].label} (${MODES[mode].unit})`,rows:[],series:[]})).join('');
      $('kinematics-chart-count').textContent=status==='loading'?'Calculating fresh results…':'Run a sweep to populate the performance graphs.';return;
    }
    const channels=data.channels.filter(relevant),plots=new Map();
    for(const channel of channels){if(!plots.has(channel.plot))plots.set(channel.plot,[]);plots.get(channel.plot).push(channel);}
    const xDefinition=data.channels.find(channel=>channel.id===xChannel);
    const xLabel=xDefinition?`${xDefinition.title} · ${xDefinition.label} (${xDefinition.unit})`:`${MODES[mode].label} (${data.input_unit})`;
    // Preserve row positions when a cross-plot X is unavailable: null breaks the
    // path in the chart renderer rather than joining across the missing sample.
    const rows=xDefinition?data.rows.map(item=>({...item,input:item.values[xDefinition.id]})):data.rows;
    grid.innerHTML=[...plots].map(([id,items])=>renderKinematicsChart({id,title:items[0].title,unit:items[0].unit,xLabel,rows,selectedIndex,series:items.map(item=>({id:item.id,label:item.label,color:COLORS[item.entity]||(item.entity.endsWith('.right')?COLORS['front.right']:COLORS['front.left']),dash:item.entity.endsWith('.right')?'6 3':''}))})).join('')||'<p class="kinematics-empty-filter">No channels match these filters.</p>';
    $('kinematics-chart-count').textContent=`${plots.size} graphs · ${channels.length} channels shown · ${data.rows.length} samples. Select a point or use the travel slider to inspect it.`;
  }
  function renderCoverage() {
    const coverage=data?.coverage??[
      {family:'Camber, toe, actuation and geometric centers',status:'unavailable',reason:'Run a sweep to determine availability from the entered geometry.'},
      {family:'Anti-effects, ARB and joint force reactions',status:'unavailable',reason:'These need additional physical inputs and separately verified models; they are not inferred from the current hardpoints.'},
    ];
    $('kinematics-coverage-list').innerHTML=coverage.map(item=>`<div class="kinematics-coverage-row"><span class="kinematics-coverage-dot ${item.status}"></span><div><strong>${escape(item.family)}</strong><p>${escape(item.reason)}</p></div><span>${item.status==='available'?'Supported':'Unavailable'}</span></div>`).join('');
    $('kinematics-assumptions').innerHTML=data?`<h4>Assumptions for this calculation</h4><ul>${data.assumptions.map(item=>`<li>${escape(item)}</li>`).join('')}</ul>${data.warnings.length?`<h4>Calculation notes</h4><ul>${data.warnings.map(item=>`<li>${escape(item)}</li>`).join('')}</ul>`:''}`:'';
  }
  function xAxisOptions(){
    const select=$('kinematics-x-axis');
    select.innerHTML='<option value="input">Sweep input</option>'+(data?GROUPS.map(group=>{const channels=data.channels.filter(channel=>channel.group===group);return channels.length?`<optgroup label="${group}">${channels.map(channel=>`<option value="${escape(channel.id)}">${escape(`${channel.title} · ${channel.label} (${channel.unit})`)}</option>`).join('')}</optgroup>`:'';}).join(''):'');
    if(!data?.channels.some(channel=>channel.id===xChannel))xChannel='input';select.value=xChannel;select.disabled=!data;
  }
  function receiveState(state){
    status=state.status;message='';data=state.status==='ready'?state.data:null;
    if(state.status==='error')message=state.error.message;
    if(data){selectedIndex=data.rows.reduce((best,item,index)=>Math.abs(item.input)<Math.abs(data.rows[best].input)?index:best,0);}
    else selectedIndex=0;
    xAxisOptions();renderStatus();selectedView();renderCharts();renderCoverage();
    if(state.status==='ready'||state.status==='error')notifyPose();
  }
  function invalidate(reason='The design or sweep changed. Run again to calculate fresh results.'){
    loader.cancel();message=reason;renderStatus();notifyPose();
  }
  function updateSettings(next){onSettingsChange(next);fingerprint=JSON.stringify(getDesign());invalidate();settingFields();readiness();}
  function select(index){if(!data||status!=='ready')return;selectedIndex=Math.max(0,Math.min(data.rows.length-1,Number(index)||0));selectedView();renderCharts();notifyPose();}
  function refreshSettings(){settingFields();readiness();}

  container.addEventListener('input',event=>{
    const element=event.target;
    if(element.id==='kinematics-position'){select(element.value);return;}
    if(['kinematics-min','kinematics-max','kinematics-samples'].includes(element.id)){
      const key=element.id.replace('kinematics-','');ranges[mode][key]=element.value===''?null:element.valueAsNumber;invalidate();return;
    }
    if(element.dataset.kinSetting){
      element.setCustomValidity('');
      const pendingKey=draftKey(element.dataset.kinSetting);
      if(!element.validity.valid){inputDrafts.set(pendingKey,{value:element.value});element.setCustomValidity('Correct or discard this unaccepted model input.');element.setAttribute('aria-invalid','true');onInputsEdited();invalidate('Complete the model input before running a new sweep.');return;}
      inputDrafts.delete(pendingKey);
      const next=settings(),key=element.dataset.kinSetting,value=element.value===''?null:element.valueAsNumber;
      if(value!==null&&!finite(value)){invalidate('Complete the model input before running a new sweep.');return;}
      if(key.startsWith('reference.'))next.reference[key.split('.')[1]]=value;
      else if(key.startsWith('forceReference.'))next.forceReference[key.split('.')[1]]=value;
      else if(key.startsWith('rockerAxis.'))next.axles[activeAxle].rockerAxis[Number(key.split('.')[1])]=value;
      else next.axles[activeAxle][key]=value;
      updateSettings(next);
    }
  });
  container.addEventListener('change',event=>{
    const element=event.target;
    if(element.dataset.kinChoice){const next=settings();next.axles[activeAxle][element.dataset.kinChoice]=element.value||null;updateSettings(next);return;}
    if(element.id==='kinematics-input-axle'){activeAxle=element.value;onAxleChange(activeAxle);settingFields();selectedView();return;}
    if(element.id==='kinematics-group')filterGroup=element.value;
    else if(element.id==='kinematics-filter-axle')filterAxle=element.value;
    else if(element.id==='kinematics-filter-side')filterSide=element.value;
    else if(element.id==='kinematics-x-axis')xChannel=element.value;
    else return;
    renderCharts();
  });
  container.addEventListener('click',event=>{
    const modeButton=event.target.closest('[data-kin-mode]');
    if(modeButton){if(mode!==modeButton.dataset.kinMode){mode=modeButton.dataset.kinMode;invalidate('Motion mode changed. Run the selected sweep.');modeFields();readiness();renderCharts();}return;}
    const point=event.target.closest('[data-sample-index]');if(point){select(point.dataset.sampleIndex);return;}
    const action=event.target.closest('[data-kin-action]')?.dataset.kinAction;
    if(action==='discard-inputs'){inputDrafts.clear();settingFields();invalidate('Unaccepted model inputs discarded. Run again with the accepted inputs.');return;}
    if(action){const next=settings();
      if(action==='front-datum'||action==='middle-datum'){next.reference={x:action==='front-datum'?0:-getDesign().wheelbase/2,y:0};for(const key of ['reference.x','reference.y'])inputDrafts.delete(draftKey(key));}
      if(action==='zero-alignment'){Object.assign(next.axles[activeAxle],{staticCamberDeg:0,staticToeInDeg:0});for(const key of ['staticCamberDeg','staticToeInDeg'])inputDrafts.delete(draftKey(key));}
      if(action==='longitudinal-axis'){next.axles[activeAxle].rockerAxis=[1,0,0];for(const key of ['rockerAxis.0','rockerAxis.1','rockerAxis.2'])inputDrafts.delete(draftKey(key));}
      settingFields();updateSettings(next);return;}
    const button=event.target.closest('button');
    if(button?.id==='kinematics-run'){
      if(readiness().length)return;
      lastRequest={design:structuredClone(getDesign()),sweep:{mode,...ranges[mode]}};
      loader.load(lastRequest);return;
    }
    if(button?.id==='kinematics-cancel'){invalidate('Calculation cancelled. Run a sweep when ready.');return;}
    if(button?.id==='kinematics-previous'){select(selectedIndex-1);return;}
    if(button?.id==='kinematics-next'){select(selectedIndex+1);return;}
    if(button?.id==='kinematics-export'&&data&&status==='ready'){
      const csv=exportKinematicsCsv({mode:data.mode,inputLabel:MODES[mode].label,inputUnit:data.input_unit,rows:data.rows,channels:data.channels,metadata:{Design:lastRequest.design.name,Model:data.model,Coordinates:data.coordinate_system,'Hardpoint frame':data.hardpoint_frame,Alignment:'Chassis-relative','Motion ratio':'Damper compression / upward wheel travel','Design snapshot JSON':JSON.stringify(lastRequest.design),'Sweep settings JSON':JSON.stringify(lastRequest.sweep),Assumptions:data.assumptions.join(' | ')}});
      const url=URL.createObjectURL(new Blob([csv],{type:'text/csv;charset=utf-8'})),anchor=document.createElement('a');anchor.href=url;anchor.download=`${getDesign().name.replace(/[^a-z0-9_-]+/gi,'-')||'suspension'}-${mode}-kinematics.csv`;anchor.click();setTimeout(()=>URL.revokeObjectURL(url),1000);
    }
  });
  modeFields();settingFields();renderStatus();selectedView();renderCharts();renderCoverage();
  return {
    sync({valid,activeAxle:nextAxle}={}){
      validDesign=valid===true;
      if(['front','rear'].includes(nextAxle))activeAxle=nextAxle;
      const next=JSON.stringify(getDesign()),changed=fingerprint&&next!==fingerprint;fingerprint=next;
      if(changed)invalidate();else if(!validDesign&&['loading','ready'].includes(status))invalidate('Correct the incomplete design inputs before recalculating.');
      settingFields();readiness();selectedView();
    },
    open(){settingFields();readiness();},
    getPose(axle){return row()?.poses?.[axle]??null;},
    invalidate,
    hasPendingInputs:()=>inputDrafts.size>0,
    resetInputs:()=>{inputDrafts.clear();settingFields();},
    refreshSettings,
  };
}
