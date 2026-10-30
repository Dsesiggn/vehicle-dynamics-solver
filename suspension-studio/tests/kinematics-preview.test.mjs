import test from 'node:test';
import assert from 'node:assert/strict';
import {newDesign} from '../src/model.js';
import {mirrorPoint} from '../src/coordinates.js';
import {suspensionPreviewGeometry} from '../src/viewer.js';

test('Backend spindle, unresolved actuator and solved U-bar reach preview without a second axle translation',()=>{
  const design=newDesign(),a=design.axles.rear;
  a.antiRollBar={type:'u-bar',enabled:true,points:{bearing:[null,null,null],bend:[80,-200,-400],arm_tip:[0,-200,-400],link_pickup:[0,-200,-280]}};
  const left={ok:true,points:structuredClone(a.hardpoints),spindle:[0,-.8,.6],rotation:[0,0,0],actuator:{ok:false},arb:{available:true,bend:[80,-200,-400],tip:[2,-200,-420],pickup:[2,-200,-300]}};
  const right={...left,points:Object.fromEntries(Object.entries(a.hardpoints).map(([k,p])=>[k,mirrorPoint(p)])),spindle:[0,.8,.6],arb:{available:true,bend:[80,200,-400],tip:[2,200,-420],pickup:[2,200,-300]}};
  const motion={ok:true,left,right},before=structuredClone(motion);
  const scene=suspensionPreviewGeometry(a,design.wheelDiameter,motion,{activeAxle:'rear',wheelbase:1600});
  assert.deepEqual(scene.corners[0].wheelAxis,left.spindle);
  assert.deepEqual(scene.corners[1].wheelAxis,right.spindle);
  assert.equal(scene.corners[0].actuatorAvailable,false);
  assert.deepEqual(scene.arb.segments[0].start,[-1598,-200,-420]);
  assert.deepEqual(scene.arb.segments[3].end,[-1598,-200,-300]);
  assert.deepEqual(scene.arb.points.find(p=>p.side==='Right'&&p.key==='arm_tip').position,[-1598,200,-420]);
  assert.deepEqual(motion,before);
});
