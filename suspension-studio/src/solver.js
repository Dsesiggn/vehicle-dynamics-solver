import { mirrorPoint, mirrorRotation, alignmentFromAxis } from './coordinates.js';
import { linkPairs, isMoving } from './model.js';
export const add = (a,b) => a.map((v,i)=>v+b[i]);
export const sub = (a,b) => a.map((v,i)=>v-b[i]);
export const scale = (a,s) => a.map(v=>v*s);
export const dot = (a,b) => a.reduce((s,v,i)=>s+v*b[i],0);
export const norm = a => Math.hypot(...a);
export const unit = a => scale(a,1/(norm(a)||1));
export const cross = (a,b) => [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
/** Rodrigues rotation: angles in radians, axle-local SAE axes (rotation vector, not Euler angles). */
export function rotate(v,r) {
  const angle = norm(r); if (angle < 1e-12) return [...v];
  const k = scale(r,1/angle), c = Math.cos(angle), s = Math.sin(angle);
  return add(add(scale(v,c),scale(cross(k,v),s)),scale(k,dot(k,v)*(1-c)));
}
function linearSolve(matrix,b) {
  const a = matrix.map((row,i)=>[...row,b[i]]), n = b.length;
  for (let j=0;j<n;j++) {
    let pivot=j; for(let i=j+1;i<n;i++) if(Math.abs(a[i][j])>Math.abs(a[pivot][j])) pivot=i;
    if(Math.abs(a[pivot][j])<1e-11) return null;
    [a[j],a[pivot]]=[a[pivot],a[j]];
    const divisor=a[j][j]; for(let k=j;k<=n;k++) a[j][k]/=divisor;
    for(let i=0;i<n;i++) if(i!==j) { const m=a[i][j]; for(let k=j;k<=n;k++) a[i][k]-=m*a[j][k]; }
  }
  return a.map(row=>row[n]);
}
function solvePose(a,bump,rack,seed) {
  const p=a.hardpoints, wc=p.wheel_center, pairs=linkPairs(a), lengths=pairs.map(([i,o])=>norm(sub(p[o],p[i])));
  const transform=(point,q)=>add(add(wc,q.slice(0,3)),rotate(sub(point,wc),q.slice(3).map(v=>v/300)));
  let normal1,normal2;
  if(a.topology==='macpherson') { const axis=unit(sub(p.strut_top,p.strut_bottom)); normal1=unit(cross(axis,[1,0,0])); normal2=unit(cross(axis,normal1)); }
  const residual=q=> {
    const result=pairs.map(([i,o],n)=>norm(sub(transform(p[o],q),add(p[i],i==='tie_rod_inner'?[0,rack,0]:[0,0,0])))-lengths[n]);
    if(a.topology==='macpherson') {
      const line=sub(p.strut_top,transform(p.strut_bottom,q)); const r=q.slice(3).map(v=>v/300);
      result.push(dot(line,rotate(normal1,r)),dot(line,rotate(normal2,r)));
    }
    result.push(q[2]+bump); return result;
  };
  let q=[...seed], error=Infinity;
  for(let iteration=0;iteration<55;iteration++) {
    const f=residual(q); error=Math.max(...f.map(Math.abs)); if(error<1e-5) break;
    const h=1e-3, jacobian=f.map(()=>[]);
    for(let j=0;j<6;j++) { const trial=[...q];trial[j]+=h;const df=residual(trial);for(let i=0;i<6;i++)jacobian[i][j]=(df[i]-f[i])/h; }
    const delta=linearSolve(jacobian,f.map(v=>-v));if(!delta) return {ok:false,error,reason:'Singular geometry: constraints do not define a unique upright pose.'};
    let accepted=false;
    for(let factor=1;factor>1/256;factor/=2) {
      const trial=q.map((v,i)=>v+delta[i]*factor), tf=residual(trial);
      if(norm(tf)<norm(f)) {q=trial;accepted=true;break;}
    }
    if(!accepted) break;
  }
  error=Math.max(...residual(q).map(Math.abs));
  if(error>1e-3 || q.some(v=>!Number.isFinite(v)) || norm(q.slice(3))>300*Math.PI/2) return {ok:false,error,reason:'No nearby assembly solution. Reduce travel or review hardpoints.'};
  const points=Object.fromEntries(Object.entries(p).map(([k,v])=>[k,isMoving(k)?transform(v,q):[...v]]));
  points.tie_rod_inner[1]+=rack;
  return {ok:true,error,q,points,rotation:q.slice(3).map(v=>v/300)};
}
function actuation(a,pose) {
  const p=a.hardpoints, out=pose.points;
  if(a.topology==='macpherson') return {ok:true,compression:norm(sub(p.strut_top,p.strut_bottom))-norm(sub(out.strut_top,out.strut_bottom))};
  if(a.actuation==='direct') return {ok:true,compression:norm(sub(p.damper_top,p.actuation_outer))-norm(sub(out.damper_top,out.actuation_outer))};
  const pivot=p.rocker_pivot, rod=sub(p.rocker_rod,pivot), damper=sub(p.rocker_damper,pivot), length=norm(sub(p.rocker_rod,p.actuation_outer));
  const position=angle=>add(pivot,rotate(rod,[angle,0,0]));
  const residual=angle=>norm(sub(position(angle),out.actuation_outer))-length;
  let angle=0;
  for(let i=0;i<35;i++) { const f=residual(angle);if(Math.abs(f)<1e-5)break;const df=(residual(angle+1e-5)-f)/1e-5;if(Math.abs(df)<1e-7)break;angle-=Math.max(-.15,Math.min(.15,f/df)); }
  if(Math.abs(residual(angle))>1e-3 || Math.abs(angle)>Math.PI/2) return {ok:false,reason:'Actuation linkage cannot reach this position.'};
  out.rocker_rod=position(angle);out.rocker_damper=add(pivot,rotate(damper,[angle,0,0]));
  return {ok:true,compression:norm(sub(p.damper_top,p.rocker_damper))-norm(sub(out.damper_top,out.rocker_damper)),angle};
}
/** Continuation from static pose keeps the Newton solve on a nearby assembly branch. */
export function solveCorner(a,bump=0,rack=0) {
  if(!Number.isFinite(bump)||!Number.isFinite(rack)||Math.abs(bump)>100||Math.abs(rack)>a.rackTravel/2+1e-8)return {ok:false,reason:'Requested motion exceeds the supported travel limits.'};
  let pose,seed=[0,0,0,0,0,0];const steps=Math.max(1,Math.ceil(Math.max(Math.abs(bump),Math.abs(rack))/4));
  for(let step=1;step<=steps;step++) {pose=solvePose(a,bump*step/steps,a.steered?rack*step/steps:0,seed);if(!pose.ok)return pose;seed=pose.q;}
  const actuator=actuation(a,pose); if(!actuator.ok)return {...pose,ok:false,reason:actuator.reason};
  return {...pose,...alignmentFromAxis(rotate([0,-1,0],pose.rotation),-1),compression:actuator.compression};
}
export function solveAxle(a,bump=0,rack=0) {
  // Solve the reflected right corner in the left template, then return BOTH
  // corners in the same SAE axle frame. Both rack ends translate in global +Y.
  const left=solveCorner(a,bump,rack),localRight=solveCorner(a,bump,-rack);
  if(!localRight.ok)return {ok:false,left,right:localRight};
  const rotation=mirrorRotation(localRight.rotation);
  const right={...localRight,rotation,
    points:Object.fromEntries(Object.entries(localRight.points).map(([key,p])=>[key,mirrorPoint(p)])),
    q:[...mirrorPoint(localRight.q.slice(0,3)),...mirrorRotation(localRight.q.slice(3))],
    ...alignmentFromAxis(rotate([0,1,0],rotation),1)};
  return {ok:left.ok&&right.ok,left,right};
}
