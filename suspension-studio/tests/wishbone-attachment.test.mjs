import test from 'node:test';
import assert from 'node:assert/strict';
import { axle, clone, generatePoints, newDesign } from '../src/model.js';
import { solveCorner, solveAxle } from '../src/solver.js';
import { mapWishbonePickup, WISHBONE_ATTACHMENT_NUMERICS } from '../src/wishbone-attachment.js';
import { buildSuspensionMembers, evaluateHeavePosition } from '../src/interference.js';

const near=(actual,expected,tolerance=1e-8)=>assert.ok(Math.abs(actual-expected)<=tolerance,`${actual} != ${expected} (tolerance ${tolerance})`);
const vectorNear=(actual,expected,tolerance=1e-8)=>actual.forEach((value,i)=>near(value,expected[i],tolerance));
const subtract=(a,b)=>a.map((value,i)=>value-b[i]);
const distance=(a,b)=>Math.hypot(...subtract(a,b));
const dot=(a,b)=>a.reduce((sum,value,i)=>sum+value*b[i],0);
const cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
const mirror=p=>[p[0],-p[1],p[2]];
const freeze=value=>{if(value&&typeof value==='object'){Object.freeze(value);Object.values(value).forEach(freeze);}return value;};

// Independent basis construction: k=(.6,0,.8), u=(0,1,0), v=(-.8,0,.6).
// A +90 degree rotation sends u to v and v to -u. No production rotation
// function is used to prescribe these integer endpoints, including the pickup
// offset 40*v from the plane containing the pivots and static ball joint.
const inclinedFixture=()=>({fore:[30,-200,-100],aft:[90,-200,-20],ballStatic:[42,-500,-84],ballSolved:[282,-200,-264],pickupStatic:[4,-350,-68]});
const inclinedExpected=[156,-240,-182];

test('independently prescribed inclined-axis rotation transports an off-plane bracket and preserves rigid geometry',()=>{
  const fixture=freeze(inclinedFixture()),before=clone(fixture),result=mapWishbonePickup(fixture);
  assert.equal(result.ok,true,result.reason);
  vectorNear(result.point,inclinedExpected);
  vectorNear(result.axis,[.6,0,.8]);near(result.angleRad,Math.PI/2);
  near(distance(result.point,fixture.fore),distance(fixture.pickupStatic,fixture.fore));
  near(distance(result.point,fixture.aft),distance(fixture.pickupStatic,fixture.aft));
  near(distance(result.point,fixture.ballSolved),distance(fixture.pickupStatic,fixture.ballStatic));
  const handedness=(ball,pickup)=>dot(cross(subtract(fixture.aft,fixture.fore),subtract(ball,fixture.fore)),subtract(pickup,fixture.fore));
  near(handedness(fixture.ballSolved,result.point),handedness(fixture.ballStatic,fixture.pickupStatic),1e-7);
  assert.ok(Math.abs(handedness(fixture.ballStatic,fixture.pickupStatic))>0,'Fixture must include a nonzero rigid bracket offset');
  assert.ok(result.reconstructionErrorMm<1e-10);assert.ok(result.rigidityErrorMm<1e-10);
  assert.deepEqual(fixture,before);
});

test('zero rotation preserves exact entered pickup and a pickup on the pivot line remains fixed',()=>{
  const fixture=inclinedFixture();fixture.ballSolved=[...fixture.ballStatic];
  const neutral=mapWishbonePickup(fixture);assert.equal(neutral.ok,true,neutral.reason);
  assert.deepEqual(neutral.point,fixture.pickupStatic);assert.notEqual(neutral.point,fixture.pickupStatic);assert.equal(neutral.angleRad,0);
  const rotating=inclinedFixture();rotating.pickupStatic=[60,-200,-60];
  const result=mapWishbonePickup(rotating);assert.equal(result.ok,true,result.reason);
  vectorNear(result.point,rotating.pickupStatic);
});

test('reversing pivot order, reflecting the corner and translating the frame preserve the physical result',()=>{
  const fixture=inclinedFixture(),reversed=mapWishbonePickup({...fixture,fore:fixture.aft,aft:fixture.fore});
  assert.equal(reversed.ok,true,reversed.reason);vectorNear(reversed.point,inclinedExpected);near(reversed.angleRad,-Math.PI/2);
  const reflected=mapWishbonePickup(Object.fromEntries(Object.entries(fixture).map(([key,value])=>[key,mirror(value)])));
  assert.equal(reflected.ok,true,reflected.reason);vectorNear(reflected.point,mirror(inclinedExpected));near(reflected.angleRad,-Math.PI/2);
  const translation=[-1530,50,-80],translated=mapWishbonePickup(Object.fromEntries(Object.entries(fixture).map(([key,value])=>[key,value.map((component,i)=>component+translation[i])])));
  assert.equal(translated.ok,true,translated.reason);vectorNear(translated.point,inclinedExpected.map((value,i)=>value+translation[i]));
});

test('incomplete or numerically degenerate arm geometry fails explicitly',()=>{
  const base={fore:[0,0,0],aft:[100,0,0],ballStatic:[20,300,0],ballSolved:[20,300,0],pickupStatic:[10,150,30]};
  for(const patch of [
    {fore:null},{ballSolved:[NaN,0,0]},{pickupStatic:[1,2]},
    {aft:[0,0,0]},{aft:[WISHBONE_ATTACHMENT_NUMERICS.minimumAxisLengthMm,0,0]},
    {ballStatic:[20,0,0]},{ballSolved:[20,0,0]},
    {ballStatic:[20,WISHBONE_ATTACHMENT_NUMERICS.minimumRadiusMm,0]},
  ]) {
    const result=mapWishbonePickup({...base,...patch});assert.equal(result.ok,false,JSON.stringify(patch));assert.match(result.reason,/Wishbone attachment/);
  }
  assert.equal(mapWishbonePickup(base,{reconstructionToleranceMm:0}).ok,false);
});

test('ball-circle reconstruction independently rejects amplified errors that individual link lengths would accept',()=>{
  const fixture={fore:[0,0,0],aft:[.1,0,0],ballStatic:[0,300,0],ballSolved:[.1,300,0],pickupStatic:[0,150,30]};
  for(const pivot of [fixture.fore,fixture.aft])assert.ok(Math.abs(distance(fixture.ballSolved,pivot)-distance(fixture.ballStatic,pivot))<1e-3);
  const result=mapWishbonePickup(fixture);
  assert.equal(result.ok,false);assert.match(result.reason,/differs from rigid arm rotation/);
});

test('the declared reconstruction tolerance is enforced without changing the solved ball or static geometry',()=>{
  const base={fore:[0,0,0],aft:[100,0,0],ballStatic:[20,300,0],ballSolved:[20,300.00099,0],pickupStatic:[10,150,30]};
  const before=clone(base),accepted=mapWishbonePickup(base);
  assert.equal(accepted.ok,true,accepted.reason);near(accepted.reconstructionErrorMm,.00099,1e-10);assert.deepEqual(accepted.point,base.pickupStatic);
  assert.equal(mapWishbonePickup(base,{reconstructionToleranceMm:1e-4}).ok,false);
  const outside={...base,ballSolved:[20,300.00101,0]};assert.equal(mapWishbonePickup(outside).ok,false);
  assert.equal(mapWishbonePickup(outside,{reconstructionToleranceMm:1e-2}).ok,true);
  assert.equal(mapWishbonePickup({...base,ballSolved:[20.00101,300,0]}).ok,false);
  assert.deepEqual(base,before);
});

function attachmentDesign(actuation='pushrod',attachment='lower-wishbone') {
  const a=newDesign().axles.front;a.actuation=actuation;a.mounting=actuation==='direct'?'direct':'bell-crank';a.actuationAttachment=attachment;a.hardpoints=generatePoints(a,460);return a;
}

test('a pure-steering fixture with fixed arm ball joints leaves both wishbone-mounted pickups fixed',()=>{
  for(const attachment of ['lower-wishbone','upper-wishbone']) {
    const a=attachmentDesign('direct',attachment);a.track=1100;
    Object.assign(a.hardpoints,{lca_outer:[0,-550,-120],uca_outer:[0,-550,-350],wheel_center:[0,-550,-230],tie_rod_outer:[80,-575,-195],actuation_outer:[0,-450,-150]});
    const original=clone(a);
    for(const rack of [-10,10]) {
      const result=solveCorner(a,0,rack);assert.equal(result.ok,true,result.reason);
      vectorNear(result.points.lca_outer,a.hardpoints.lca_outer,1e-4);
      vectorNear(result.points.uca_outer,a.hardpoints.uca_outer,1e-4);
      vectorNear(result.points.actuation_outer,a.hardpoints.actuation_outer,1e-4);
      assert.ok(Math.abs(result.steer)>1,'Fixture must steer rather than remain at neutral');
      const upright=solveCorner({...a,actuationAttachment:'upright'},0,rack);assert.equal(upright.ok,true,upright.reason);
      assert.ok(distance(upright.points.actuation_outer,a.hardpoints.actuation_outer)>1,'An upright-mounted pickup must move in this fixture');
    }
    assert.deepEqual(a,original);
  }
});

test('fixed wheel-center height does not freeze the arm when coupled steering geometry moves its ball joint',()=>{
  const a=attachmentDesign('direct','lower-wishbone'),result=solveCorner(a,0,15);
  assert.equal(result.ok,true,result.reason);near(result.points.wheel_center[2],a.hardpoints.wheel_center[2],1e-6);
  assert.ok(distance(result.points.lca_outer,a.hardpoints.lca_outer)>.05);
  assert.ok(distance(result.points.actuation_outer,a.hardpoints.actuation_outer)>.03);
  near(distance(result.points.actuation_outer,result.points.lca_outer),distance(a.hardpoints.actuation_outer,a.hardpoints.lca_outer),1e-3);
});

test('combined bump and steer preserve pickup attachment and actuator closure for both corners and all actuation methods',()=>{
  for(const attachment of ['lower-wishbone','upper-wishbone'])for(const actuation of ['pushrod','pullrod','direct']) {
    const a=attachmentDesign(actuation,attachment),original=clone(a),prefix=attachment==='lower-wishbone'?'lca':'uca';
    for(const [bump,rack] of [[-15,-8],[0,0],[15,8]]) {
      const solved=solveAxle(a,bump,rack);assert.equal(solved.ok,true,`${attachment}/${actuation}: ${solved.left?.reason||solved.right?.reason}`);
      const opposite=solveAxle(a,bump,-rack);assert.equal(opposite.ok,true);
      for(const side of ['left','right']) {
        const p=solved[side].points,staticPoints=Object.fromEntries(Object.entries(a.hardpoints).map(([key,value])=>[key,side==='left'?value:mirror(value)]));
        for(const key of [`${prefix}_front`,`${prefix}_rear`,`${prefix}_outer`])near(distance(p.actuation_outer,p[key]),distance(staticPoints.actuation_outer,staticPoints[key]),1e-3);
        assert.ok(solved[side].actuationAttachmentResidualMm<=1e-3);
        if(actuation==='direct')near(solved[side].compression,distance(staticPoints.damper_top,staticPoints.actuation_outer)-distance(p.damper_top,p.actuation_outer),1e-8);
        else {
          near(distance(p.actuation_outer,p.rocker_rod),distance(staticPoints.actuation_outer,staticPoints.rocker_rod),1e-3);
          near(solved[side].compression,distance(staticPoints.damper_top,staticPoints.rocker_damper)-distance(p.damper_top,p.rocker_damper),1e-8);
        }
      }
      vectorNear(solved.right.points.actuation_outer,mirror(opposite.left.points.actuation_outer));
      vectorNear(solved.right.points.rocker_rod||solved.right.points.damper_top,mirror(opposite.left.points.rocker_rod||opposite.left.points.damper_top));
      const upright=solveAxle({...a,actuationAttachment:'upright'},bump,rack);assert.equal(upright.ok,true);
      for(const side of ['left','right'])for(const key of Object.keys(a.hardpoints)) {
        if(!['actuation_outer','rocker_rod','rocker_damper'].includes(key))assert.deepEqual(solved[side].points[key],upright[side].points[key],`${key} pose changed with attachment`);
      }
    }
    assert.deepEqual(a,original);
  }
});

test('legacy missing attachment reproduces explicit upright poses and unsupported bodies fail before solving',()=>{
  for(const topology of ['double-wishbone','macpherson','multi-link']) {
    const a=axle(topology),legacy=clone(a);delete legacy.actuationAttachment;
    assert.deepEqual(solveAxle(legacy,12,7),solveAxle(a,12,7));
    if(topology!=='double-wishbone')for(const attachment of ['lower-wishbone','upper-wishbone']) {
      const result=solveCorner({...a,actuationAttachment:attachment},0,0);assert.equal(result.ok,false);assert.match(result.reason,/unsupported/);
    }
  }
  for(const value of [null,undefined,'unknown',false])assert.equal(solveCorner({...axle(),actuationAttachment:value},0,0).ok,false);
});

test('heave interference uses selected-wishbone pickup and damper endpoints on both corners rather than legacy upright endpoints',()=>{
  for(const attachment of ['lower-wishbone','upper-wishbone'])for(const actuation of ['pushrod','pullrod','direct'])for(const activeAxle of ['front','rear']) {
    const design=newDesign(),a=attachmentDesign(actuation,attachment);design.axles[activeAxle]=a;design.interference.linkOD=20;
    const result=evaluateHeavePosition({axle:a,design,activeAxle,travel:15});
    assert.equal(result.ok,true,`${attachment}/${actuation}/${activeAxle}: ${result.reason}`);
    assert.deepEqual(result.members,buildSuspensionMembers(a,result.solved,20));
    const legacy=solveAxle({...a,actuationAttachment:'upright'},15,0);assert.equal(legacy.ok,true);
    for(const side of ['left','right']) {
      const p=result.solved[side].points,old=legacy[side].points;
      const pickupMember=result.members.find(member=>member.side===side&&(member.start==='actuation_outer'||member.end==='actuation_outer'));
      assert.ok(pickupMember,`${side} must include the actuation pickup in a checked member`);
      assert.deepEqual(pickupMember.p1,p[pickupMember.start]);assert.deepEqual(pickupMember.p2,p[pickupMember.end]);
      const checkedPickup=pickupMember.start==='actuation_outer'?pickupMember.p1:pickupMember.p2;
      assert.ok(distance(checkedPickup,old.actuation_outer)>1e-3,'Fixture must distinguish arm-mounted and upright-mounted endpoints');
      const damper=result.members.find(member=>member.side===side&&member.kind==='damper');
      assert.ok(damper);assert.deepEqual(damper.p1,p[damper.start]);assert.deepEqual(damper.p2,p[damper.end]);
      assert.ok(distance(damper.p2,old[damper.end])>1e-3,'Damper endpoint must respond to the selected pickup attachment');
    }
  }
});
