// Optional v2 extension. Unknown measurements remain null; no hardpoint changes.
export function newKinematicsSettings() {
  const axle = () => ({staticCamberDeg:null,staticToeInDeg:null,rockerAxis:[null,null,null],brakeReaction:null,driveReaction:null,arbModel:null,arbAttachment:null});
  return {reference:{x:null,y:null},forceReference:{cgHeightMm:null,frontBrakeFraction:null,frontDriveFraction:null},contactModel:'rigid-circle',axles:{front:axle(),rear:axle()}};
}
export function completeKinematicsSettings(value) {
  const defaults=newKinematicsSettings();if(value===undefined)return defaults;
  const copy=structuredClone(value);
  return {...defaults,...copy,forceReference:{...defaults.forceReference,...copy.forceReference},axles:Object.fromEntries(['front','rear'].map(name=>[name,{...defaults.axles[name],...copy.axles[name]}]))};
}
export function validateKinematicsSettings(value) {
  if(value===undefined)return [];
  const errors=[],record=v=>v&&typeof v==='object'&&!Array.isArray(v);
  if(!record(value)||!record(value.reference)||!record(value.axles))return ['Kinematics settings must contain reference coordinates and both axle definitions.'];
  const optional=(v,limit,label)=>{if(v!==null&&(!Number.isFinite(v)||Math.abs(v)>limit))errors.push(`${label} must be blank or a finite number within ±${limit}.`);};
  for(const key of ['x','y'])optional(value.reference[key],20000,`Rig reference ${key.toUpperCase()}`);
  if(value.contactModel!=='rigid-circle')errors.push('Unsupported kinematic tire contact model.');
  if(value.forceReference!==undefined) {
    if(!record(value.forceReference))errors.push('Anti-effect force reference must be an object.');
    else for(const [key,max] of [['cgHeightMm',10000],['frontBrakeFraction',1],['frontDriveFraction',1]]) {
      const v=value.forceReference[key];
      if(v!==null&&(!Number.isFinite(v)||v<0||v>max||key==='cgHeightMm'&&v===0))errors.push(`${key} must be blank or ${key==='cgHeightMm'?'positive up to 10000 mm':'between 0 and 1'}.`);
    }
  }
  for(const name of ['front','rear']) {
    const a=value.axles[name];
    if(!record(a)){errors.push(`${name}: missing kinematic inputs.`);continue;}
    optional(a.staticCamberDeg,45,`${name} static camber`);optional(a.staticToeInDeg,45,`${name} static toe-in`);
    for(const key of ['brakeReaction','driveReaction'])if(a[key]!=null&&!['upright','chassis'].includes(a[key]))errors.push(`${name}: invalid ${key}.`);
    if(a.arbModel!=null&&a.arbModel!=='straight-transverse')errors.push(`${name}: invalid ARB model.`);
    if(a.arbAttachment!=null&&!['upright','upper-wishbone','lower-wishbone','rocker'].includes(a.arbAttachment))errors.push(`${name}: invalid ARB pickup attachment.`);
    if(!Array.isArray(a.rockerAxis)||a.rockerAxis.length!==3)errors.push(`${name}: rocker axis needs three direction components.`);
    else {
      a.rockerAxis.forEach(v=>optional(v,10000,`${name} rocker-axis component`));
      if(a.rockerAxis.every(Number.isFinite)&&Math.hypot(...a.rockerAxis)<1e-9)errors.push(`${name}: rocker axis cannot be a zero vector.`);
    }
  }
  return errors;
}
