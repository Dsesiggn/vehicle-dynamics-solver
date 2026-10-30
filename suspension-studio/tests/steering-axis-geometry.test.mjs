import test from 'node:test';
import assert from 'node:assert/strict';
import {steeringAxisGeometry, STEERING_AXIS_MIN_LENGTH_MM} from '../src/steering-axis.js';
import {newDesign, axle, clone} from '../src/model.js';
import {suspensionPreviewGeometry} from '../src/viewer.js';

const near=(actual,expected,tolerance=1e-12)=>assert.ok(Math.abs(actual-expected)<=tolerance,`${actual} != ${expected}`);
const vectorNear=(actual,expected,tolerance=1e-12)=>actual.forEach((value,index)=>near(value,expected[index],tolerance));
const mirror=([x,y,z])=>[x,-y,z];
const freeze=value=>{if(value&&typeof value==='object'){Object.freeze(value);Object.values(value).forEach(freeze);}return value;};

test('double-wishbone axis joins the outer ball joints with its full three-dimensional direction',()=>{
  const points=freeze({lca_outer:[30,-500,-100],uca_outer:[60,-460,-220],wheel_center:[0,-600,-210],strut_top:[500,500,500]});
  const before=clone(points),result=steeringAxisGeometry('double-wishbone',points);
  assert.equal(result.available,true,result.reason);
  assert.equal(result.lowerKey,'lca_outer');assert.equal(result.upperKey,'uca_outer');
  assert.deepEqual(result.lower,points.lca_outer);assert.deepEqual(result.upper,points.uca_outer);
  // Difference [30,40,-120] has independently known length 130 mm.
  vectorNear(result.direction,[3/13,4/13,-12/13]);near(Math.hypot(...result.direction),1);
  assert.notEqual(result.lower,points.lca_outer);assert.notEqual(result.upper,points.uca_outer);
  assert.deepEqual(points,before);
});

test('MacPherson steering axis uses the lower ball joint and upper bearing rather than the strut shaft',()=>{
  const points={lca_outer:[10,-500,-100],strut_top:[40,-460,-220],strut_bottom:[70,-480,-170]};
  const result=steeringAxisGeometry('macpherson',points);
  assert.equal(result.available,true,result.reason);assert.equal(result.lowerKey,'lca_outer');assert.equal(result.upperKey,'strut_top');
  assert.deepEqual(result.lower,points.lca_outer);assert.deepEqual(result.upper,points.strut_top);
  vectorNear(result.direction,[3/13,4/13,-12/13]);
  const shaft=points.strut_top.map((value,index)=>value-points.strut_bottom[index]);
  const cosine=Math.abs(result.direction.reduce((sum,value,index)=>sum+value*shaft[index],0)/Math.hypot(...shaft));
  assert.ok(cosine<.9,'Fixture must distinguish shaft and kingpin directions');
});

test('reflection and common front-origin translation preserve the defining segment and correct direction',()=>{
  const points={lca_outer:[30,-500,-100],uca_outer:[60,-460,-220]};
  const base=steeringAxisGeometry('double-wishbone',points);
  const reflected=steeringAxisGeometry('double-wishbone',Object.fromEntries(Object.entries(points).map(([key,value])=>[key,mirror(value)])));
  vectorNear(reflected.direction,mirror(base.direction));assert.deepEqual(reflected.lower,mirror(base.lower));
  const shifted=steeringAxisGeometry('double-wishbone',Object.fromEntries(Object.entries(points).map(([key,value])=>[key,[value[0]-1530,value[1],value[2]]])));
  assert.deepEqual(shifted.lower,[-1500,-500,-100]);assert.deepEqual(shifted.upper,[-1470,-460,-220]);
  vectorNear(shifted.direction,base.direction);
});

test('unsupported topology and undefined, nonfinite or unresolved endpoints are explicitly unavailable',()=>{
  const points={lca_outer:[0,-500,-100],uca_outer:[0,-500,-300]};
  for(const topology of ['multi-link','unknown',null]) {
    const result=steeringAxisGeometry(topology,points);assert.equal(result.available,false);assert.ok(result.reason.length>0);
    assert.equal(result.direction,undefined);
  }
  for(const patch of [undefined,{}, {...points,uca_outer:null},{...points,uca_outer:[0,NaN,0]},
    {...points,uca_outer:[0,Infinity,0]},{...points,lca_outer:[0,0]},{...points,uca_outer:points.lca_outer},
    {lca_outer:[0,0,0],uca_outer:[STEERING_AXIS_MIN_LENGTH_MM,0,0]}]) {
    const result=steeringAxisGeometry('double-wishbone',patch);assert.equal(result.available,false);assert.ok(result.reason.length>0);
    assert.equal(result.direction,undefined);
  }
  assert.equal(steeringAxisGeometry('double-wishbone',{lca_outer:[0,0,0],uca_outer:[2*STEERING_AXIS_MIN_LENGTH_MM,0,0]}).available,true);
});

test('preview axis consumes current solved pivots on both corners and translates rear endpoints once',()=>{
  for(const topology of ['double-wishbone','macpherson']) {
    const a=axle(topology),upperKey=topology==='double-wishbone'?'uca_outer':'strut_top';
    const left=clone(a.hardpoints);left.lca_outer=[15,-520,-90];
    if(topology==='double-wishbone')left[upperKey]=[45,-480,-210];
    const right=Object.fromEntries(Object.entries(left).map(([key,value])=>[key,mirror(value)]));
    const motion={ok:true,left:{points:left,rotation:[.01,.02,.03]},right:{points:right,rotation:[-.01,.02,-.03]}};
    const before=clone({a,motion}),context={activeAxle:'rear',wheelbase:1530,obstacles:{}};
    const scene=suspensionPreviewGeometry(a,460,motion,context);
    for(const [index,points] of [[0,left],[1,right]]) {
      const axis=scene.corners[index].steeringAxis;assert.equal(axis.available,true,axis.reason);
      const front=point=>[point[0]-1530,point[1],point[2]];
      assert.deepEqual(axis.lower,front(points.lca_outer));assert.deepEqual(axis.upper,front(points[upperKey]));
      vectorNear(axis.direction,steeringAxisGeometry(topology,points).direction);
    }
    assert.deepEqual({a,motion},before);
  }
});

test('static and failed-motion preview retain accepted pivots, while multi-link has no invented axis',()=>{
  const d=newDesign(),a=d.axles.front,context={activeAxle:'front',wheelbase:d.wheelbase,obstacles:{}};
  for(const motion of [null,{ok:false,left:{points:{lca_outer:[999,999,999]}}}]) {
    const scene=suspensionPreviewGeometry(a,d.wheelDiameter,motion,context);
    assert.deepEqual(scene.corners[0].steeringAxis.lower,a.hardpoints.lca_outer);
    assert.deepEqual(scene.corners[0].steeringAxis.upper,a.hardpoints.uca_outer);
    assert.deepEqual(scene.corners[1].steeringAxis.lower,mirror(a.hardpoints.lca_outer));
  }
  const multilink=suspensionPreviewGeometry(axle('multi-link'),460,null,context);
  assert.ok(multilink.corners.every(corner=>corner.steeringAxis.available===false&&corner.steeringAxis.reason.includes('virtual-axis')));
});
