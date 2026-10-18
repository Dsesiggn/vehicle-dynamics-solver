import test from 'node:test';
import assert from 'node:assert/strict';
import {buildSteeringScene, includedAngleArcOffsets, isScenePoint, saeToThree, sceneBounds, wheelRingOffsets} from '../src/steering-scene.js';
import {exampleSteeringDesign} from '../src/steering.js';
import {steeringConstruction} from '../src/steering-geometry.js';

const close=(a,b,tol=1e-10)=>assert.ok(Math.abs(a-b)<tol,`${a} != ${b}`);
const vclose=(a,b,tol=1e-10)=>a.forEach((v,i)=>close(v,b[i],tol));
const dot=(a,b)=>a.reduce((s,v,i)=>s+v*b[i],0);
const cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
const sub=(a,b)=>a.map((v,i)=>v-b[i]);
const unit=a=>a.map(v=>v/Math.hypot(...a));
const lookup=(items,id)=>items.find(item=>item.id===id);

// The renderer contract is tested with response-shaped data, deliberately
// independent of any steering solve. A viewer must display exactly these values.
function poseFixture(){
  const design=exampleSteeringDesign();
  design.left.axisUpper=[30,-530,-450];design.left.axisLower=[-10,-560,-150];
  design.left.tieOuter=[-100,-520,-250];design.left.tieInner=[-110,-250,-180];
  const result={ok:true,left:{outer:[-87,-509,-246],inner:[-110,-263,-180],wheelCenter:[8,-601,-240],spindle:unit([-.12,-.98,.13])},right:{outer:[-102,541,-259],inner:[-100,237,-250],wheelCenter:[-4,602,-251],spindle:unit([-.16,.97,.07])},metrics:{turn:'right',insideSide:'right',insideDeg:11,outsideDeg:8,idealOutsideDeg:9}};
  result.construction=steeringConstruction(design,result);
  return {design,result};
}

test('SAE display adapter is a proper rotation preserving geometry and heading signs',()=>{
  vclose(saeToThree([1,2,3]),[2,-3,-1]);
  const a=[3,4,-9],b=[-2,7,5];
  close(dot(saeToThree(a),saeToThree(b)),dot(a,b));
  vclose(cross(saeToThree(a),saeToThree(b)),saeToThree(cross(a,b)));
  const ex=saeToThree([1,0,0]),ey=saeToThree([0,1,0]),ez=saeToThree([0,0,1]);
  close(dot(ex,cross(ey,ez)),1);
  const h=.3;vclose(saeToThree([Math.cos(h),Math.sin(h),0]),[Math.sin(h),0,-Math.cos(h)]);
  assert.equal(isScenePoint(Array(3)),false);
});

test('backend XYZ joints, wheel centers, spindle and construction remain unflattened and unmodified',()=>{
  const {design,result}=poseFixture(),before=JSON.stringify({design,result}),scene=buildSteeringScene(design,result);
  assert.equal(scene.pose,'solved');
  for(const side of ['left','right']){
    assert.deepEqual(lookup(scene.segments,`${side}:tie-rod`).points,[result[side].inner,result[side].outer]);
    assert.deepEqual(lookup(scene.segments,`${side}:effective-arm`).points,[result.construction[side].axisFoot,result[side].outer]);
    assert.equal(lookup(scene.segments,`${side}:effective-arm`).style,'radius');
    const ring=lookup(scene.rings,`${side}:wheel-plane`);
    assert.deepEqual(ring.center,result[side].wheelCenter);assert.deepEqual(ring.normal,result[side].spindle);
    assert.notEqual(ring.center[2],0);
    assert.ok(ring.unitOffsets.some(p=>Math.abs(p[2])>.5));
    ring.unitOffsets.forEach(p=>{close(dot(p,ring.normal),0);close(Math.hypot(...p),1);});
  }
  assert.equal(JSON.stringify({design,result}),before);
  lookup(scene.segments,'left:tie-rod').points[0][2]=999;
  assert.equal(result.left.inner[2],-180,'scene arrays must not alias response geometry');
});

test('symbolic wheel ring has the given normal and positive camber tilts the top outward on either side',()=>{
  const d=exampleSteeringDesign(),camber=15*Math.PI/180;
  d.left.staticCamberDeg=15;d.right.staticCamberDeg=15;
  const scene=buildSteeringScene(d,{ok:false,construction:steeringConstruction(d)});
  for(const side of ['left','right']){
    const s=side==='left'?-1:1,ring=lookup(scene.rings,`${side}:wheel-plane`),normal=saeToThree(ring.normal),up=[0,1,0];
    vclose(normal,[s*Math.cos(camber),-Math.sin(camber),0]);
    const top=unit(up.map((v,i)=>v-dot(up,normal)*normal[i]));
    assert.ok(top[0]*s>0,'wheel top tilts toward occupant-side exterior');
    ring.unitOffsets.forEach(p=>close(dot(saeToThree(p),normal),0));
  }
  assert.deepEqual(wheelRingOffsets([0,0,0]),[]);
});

test('plan arm, rods, intersection and alpha are at Z=0 while beta stays in its actual spatial plane',()=>{
  const {design,result}=poseFixture(),scene=buildSteeringScene(design,result,{fitIntersection:true});
  for(const line of scene.segments.filter(item=>item.style==='projection'))line.points.forEach(p=>assert.equal(p[2],0));
  assert.equal(lookup(scene.markers,'arm-intersection:plan').position[2],0);
  for(const side of ['left','right']){
    const part=result.construction[side],alpha=lookup(scene.arcs,`${side}:alpha`),beta=lookup(scene.arcs,`${side}:beta`);
    assert.equal(alpha.center[2],0);alpha.unitOffsets.forEach(p=>assert.equal(p[2],0));
    assert.deepEqual(beta.center,result[side].outer);
    const rayA=unit(sub(part.axisFoot,part.outer)),rayB=unit(sub(part.inner,part.outer)),normal=unit(cross(rayA,rayB));
    vclose(beta.unitOffsets[0],rayA);vclose(beta.unitOffsets.at(-1),rayB);
    beta.unitOffsets.forEach(p=>close(dot(p,normal),0));
    assert.ok(beta.unitOffsets.some(p=>Math.abs(p[2])>1e-3));
    assert.equal(beta.angleDeg,part.spatialAngleDeg);
  }
  // An undefined arc plane never receives an arbitrary semicircle.
  assert.deepEqual(includedAngleArcOffsets([1,0,0],[-1,0,0]),[]);
});

test('backend construction is used as supplied; solved pose does not regenerate missing construction',()=>{
  const {design,result}=poseFixture();result.construction.left.planAngleDeg=123.456;result.construction.left.spatialAngleDeg=78.901;
  const scene=buildSteeringScene(design,result);
  close(lookup(scene.arcs,'left:alpha').angleDeg,123.456);close(lookup(scene.arcs,'left:beta').angleDeg,78.901);
  delete result.construction;
  const absent=buildSteeringScene(design,result);
  assert.equal(lookup(absent.arcs,'left:alpha'),undefined);assert.equal(lookup(absent.arcs,'left:beta'),undefined);assert.equal(absent.intersection,null);
});

test('failed row is static only, pending clears all geometry, and rear markers do not invent wheel height or tire size',()=>{
  const {design,result}=poseFixture(),failed={ok:false,left:result.left,right:result.right,construction:steeringConstruction(design)};
  const scene=buildSteeringScene(design,failed,{frontOnly:false});
  assert.equal(scene.pose,'static');assert.deepEqual(lookup(scene.segments,'left:tie-rod').points,[design.left.tieInner,design.left.tieOuter]);
  assert.equal(scene.headings.length,0);
  const rear=lookup(scene.segments,'rear-axle:reference');rear.points.forEach(p=>assert.equal(p[2],0));
  assert.deepEqual(rear.points,[[-design.wheelbase,-design.rearTrack/2,0],[-design.wheelbase,design.rearTrack/2,0]]);
  assert.ok(scene.rings.every(r=>r.id.startsWith('left:')||r.id.startsWith('right:')));
  assert.equal(lookup(buildSteeringScene(design,result).segments,'rear-axle:reference'),undefined);
  const pending=buildSteeringScene(design,result,{ready:false});
  assert.equal(pending.pose,'empty');for(const key of ['segments','markers','rings','arcs','labels','headings','boundsPoints'])assert.equal(pending[key].length,0);
});

test('distant intersection enters fit only when requested and never changes physical geometry',()=>{
  const {design,result}=poseFixture();result.construction.intersection={status:'finite',point:[-1e10,3e9,0],rearAxleOffsetMm:-1e10+design.wheelbase};
  const ordinary=buildSteeringScene(design,result),fit=buildSteeringScene(design,result,{fitIntersection:true});
  assert.ok(sceneBounds(ordinary.boundsPoints).min[0]>-1e4);
  assert.equal(sceneBounds(fit.boundsPoints).min[0],-1e10);
  assert.deepEqual(lookup(ordinary.segments,'left:tie-rod'),lookup(fit.segments,'left:tie-rod'));
  assert.deepEqual(lookup(ordinary.markers,'arm-intersection:plan').position,[-1e10,3e9,0]);
});

test('nearly horizontal axis remote K is clipped without overwhelming normal assembly bounds',()=>{
  const d=exampleSteeringDesign();
  d.left.axisUpper=[-100,-500,-250];d.left.axisLower=[100,-500,-250+1e-7];d.left.tieOuter=[0,-520,-240];
  const construction=steeringConstruction(d),scene=buildSteeringScene(d,{ok:false,construction});
  assert.ok(Math.abs(construction.left.planAxisPoint[0])>1e9);
  const bounds=sceneBounds(scene.boundsPoints);
  assert.ok(Math.max(...bounds.min.map(Math.abs),...bounds.max.map(Math.abs))<1e4);
  assert.equal(lookup(scene.segments,'left:drop-axis-height'),undefined);
  for(const line of scene.segments.filter(item=>item.id.startsWith('left:')&&item.style==='projection'))for(const p of line.points)assert.ok(Math.max(...p.map(Math.abs))<1e4);
});
