import { COORDINATE_SYSTEM, legacyToSAE, mirrorPoint, mirrorRotation, alignmentFromAxis, toDisplay } from '../src/coordinates.js';
import { importDesign, readLibrary, STORAGE, LEGACY_STORAGE } from '../src/migration.js';
import {newDesign,preset,axle,generatePoints,normalizeAxle,changeDimensions,validateDesign,linkPairs,clone} from '../src/model.js';
import {solveCorner,solveAxle,norm,sub,rotate,cross} from '../src/solver.js';
export function runChecks() {
  const results=[];
  const assert=(condition,message)=>{if(!condition)throw Error(message);};
  const near=(a,b,tolerance=1e-3)=>assert(Math.abs(a-b)<tolerance,`${a} differs from ${b}`);
  const test=(name,fn)=>{try{fn();results.push({name,ok:true});}catch(e){results.push({name,ok:false,error:e.message});}};
  for(const id of ['formula','road','cr26'])test(`${id} preset passes schema validation`,()=>assert(validateDesign(preset(id)).length===0,validateDesign(preset(id)).join('; ')));
  for(const topology of ['double-wishbone','macpherson','multi-link']) {
    const a=axle(topology);
    test(`${topology}: static pose is unchanged`,()=>{const r=solveCorner(a);assert(r.ok,r.reason);near(r.camber,0);near(r.toe,0);near(r.compression,0);});
    for(const bump of [-30,30])test(`${topology}: ${bump} mm bump preserves links`,()=>{
      const r=solveCorner(a,bump);assert(r.ok,r.reason);near(r.points.wheel_center[2]-a.hardpoints.wheel_center[2],-bump);
      for(const [i,o] of linkPairs(a))near(norm(sub(r.points[o],r.points[i])),norm(sub(a.hardpoints[o],a.hardpoints[i])));
      assert(Math.abs(r.camber)>.01,'camber should respond to unequal geometry');
    });
    test(`${topology}: combined bump and steer solves both sides`,()=>{const r=solveAxle(a,15,12);assert(r.ok,r.left.reason||r.right.reason);assert(r.left.toe*r.right.toe<0,'toe-in should have opposite signs for steering');assert(r.left.error<.001&&r.right.error<.001,'constraint tolerance');});
  }
  for(const actuation of ['pushrod','pullrod','direct'])test(`${actuation}: actuator motion remains constrained`,()=>{
    const a=axle();a.actuation=actuation;normalizeAxle(a);a.hardpoints=generatePoints(a,460);const r=solveCorner(a,20);assert(r.ok,r.reason);assert(Math.abs(r.compression)>.1,'damper must move');
    if(actuation!=='direct')near(norm(sub(r.points.rocker_rod,r.points.actuation_outer)),norm(sub(a.hardpoints.rocker_rod,a.hardpoints.actuation_outer)));
  });
  test('CR26 measurements are converted to SAE coordinates',()=>{const p=preset('cr26').axles.front.hardpoints;near(p.uca_front[0],127);near(p.uca_front[1],-263.53);near(p.uca_front[2],-263.53);near(p.tie_rod_outer[0],73.03);near(p.wheel_center[1],-558.8);assert(solveCorner(preset('cr26').axles.front,15).ok,'CR26 bump solve');});
  test('Front and rear configurations are independent',()=>{const d=newDesign();changeDimensions(d.axles.front,'track',1300);near(d.axles.rear.track,1180);near(d.axles.front.hardpoints.wheel_center[1],-650);near(-d.axles.front.hardpoints.tie_rod_inner[1]*2,d.axles.front.rackLength);});
  test('Rack length updates its hardpoint',()=>{const a=axle();changeDimensions(a,'rackLength',510);near(a.hardpoints.tie_rod_inner[1],-255);});
  test('Non-steered axle ignores rack displacement',()=>{const a=axle('multi-link',1200,false);const x=solveCorner(a,10,0),y=solveCorner(a,10,20);assert(x.ok&&y.ok,'solve');near(x.toe,y.toe);});
  test('JSON round trip retains geometry and choices',()=>{const d=preset('road');assert(JSON.stringify(JSON.parse(JSON.stringify(d)))===JSON.stringify(d),'round trip');});
  test('Missing hardpoints and incompatible schema are rejected',()=>{const d=newDesign();delete d.axles.front.hardpoints.lca_outer;assert(validateDesign(d).length>0,'missing point');d.version=999;assert(validateDesign(d).length>0,'version');});
  test('Nonfinite input and incompatible actuation are rejected',()=>{const d=newDesign();d.axles.front.track=NaN;d.axles.rear.mounting='direct';assert(validateDesign(d).length>=2,'validation');});
  test('Extra point names and nonboolean steering are rejected',()=>{const d=newDesign();d.axles.front.hardpoints.unexpected=[0,0,0];d.axles.front.steered='yes';assert(validateDesign(d).length>=2,'validation');});
  test('Track / hardpoint disagreement is rejected',()=>{const d=newDesign();d.axles.front.hardpoints.wheel_center[1]=800;assert(validateDesign(d).length>0,'consistency');});
  test('Degenerate linkage fails explicitly',()=>{const a=axle();for(const [k,p] of Object.entries(a.hardpoints))a.hardpoints[k]=[0,p[1],-230];const r=solveCorner(a,30);assert(!r.ok,'must not claim a solved pose');});
  test('Out-of-range motion fails explicitly',()=>{assert(!solveCorner(axle(),0,500).ok,'rack limit');assert(!solveCorner(axle(),NaN,0).ok,'NaN');});
  test('Validation rejects null and malformed inputs without crashing',()=>{for(const d of [null,{},[],{version:1,axles:{}}])assert(validateDesign(d).length>0,'malformed');});
  test('Malformed coordinate types are rejected without throwing',()=>{for(const point of [null, 'bad', 12, {}, [1,2], [1,NaN,3]]) {const d=newDesign();d.axles.front.hardpoints.lca_front=point;assert(validateDesign(d).length>0,'coordinate type');}});
  test('SAE basis is right-handed and display conversion preserves directions',()=>{
    assert(JSON.stringify(cross(legacyToSAE([1,0,0]),legacyToSAE([0,1,0])))===JSON.stringify(legacyToSAE([0,0,1])),'handedness');
    near(toDisplay([0,0,1])[2],-1);near(toDisplay([1,0,0])[1],-1);
  });
  test('Alignment separates SAE steer from left/right toe-in',()=>{
    const radians=Math.PI/180;
    for(const side of [-1,1]) {
      const steer=alignmentFromAxis(rotate([0,side,0],[0,0,5*radians]),side);
      near(steer.steer,5);near(steer.toe,-side*5);near(steer.camber,0);
      const camber=alignmentFromAxis(rotate([0,side,0],[side*3*radians,0,0]),side);
      near(camber.camber,3);near(camber.toe,0);
    }
  });
  test('Reflection handles rotation vectors as axial vectors',()=>{
    const v=[12,-5,7],r=[.1,.2,-.3];
    const expected=mirrorPoint(rotate(v,r)),actual=rotate(mirrorPoint(v),mirrorRotation(r));
    expected.forEach((n,i)=>near(actual[i],n,1e-10));
  });
  for(const topology of ['double-wishbone','macpherson','multi-link'])test(`${topology}: both solved corners use global SAE coordinates`,()=>{
    const a=axle(topology),r=solveAxle(a,20,12);assert(r.ok,'solve');
    near(r.left.points.tie_rod_inner[1]-a.hardpoints.tie_rod_inner[1],12);
    near(r.right.points.tie_rod_inner[1]+a.hardpoints.tie_rod_inner[1],12);
    near(r.left.points.wheel_center[2]-a.hardpoints.wheel_center[2],-20);
    near(r.right.points.wheel_center[2]-a.hardpoints.wheel_center[2],-20);
    assert(r.left.points.wheel_center[1]<0&&r.right.points.wheel_center[1]>0,'sides');
    assert(r.left.steer*r.right.steer>0,'both steer in same physical direction');
    for(const [i,o] of linkPairs(a))near(norm(sub(r.right.points[o],r.right.points[i])),norm(sub(a.hardpoints[o],a.hardpoints[i])));
    const symmetric=solveAxle(a,20,0);assert(symmetric.ok,'symmetric solve');
    near(symmetric.left.camber,symmetric.right.camber);near(symmetric.left.toe,symmetric.right.toe);
  });
  const legacyDesign=()=>{
    const d=preset('cr26');d.version=1;delete d.coordinateSystem;delete d.hardpointFrame;delete d.lengthUnit;
    for(const a of Object.values(d.axles))for(const [k,p] of Object.entries(a.hardpoints))a.hardpoints[k]=legacyToSAE(p);
    return d;
  };
  test('v1 migration preserves geometry without mutating input or double conversion',()=>{
    const old=legacyDesign(),snapshot=JSON.stringify(old),first=importDesign(old);
    assert(first.migrated,'migration');assert(JSON.stringify(old)===snapshot,'input mutated');
    assert(JSON.stringify(first.design.axles)===JSON.stringify(preset('cr26').axles),'geometry');
    const second=importDesign(first.design);assert(!second.migrated,'double migration');
    assert(JSON.stringify(second.design)===JSON.stringify(first.design),'idempotence');
  });
  test('Unknown or ambiguous axes, units and versions are rejected at import',()=>{
    const values=[{...newDesign(),coordinateSystem:'ISO_Z_UP'},{...newDesign(),lengthUnit:'m'},{...newDesign(),hardpointFrame:'CG'},{...newDesign(),version:3},{...legacyDesign(),coordinateSystem:COORDINATE_SYSTEM}];
    for(const d of values){let rejected=false;try{importDesign(d);}catch{rejected=true;}assert(rejected,'unsupported metadata');}
  });
  test('Saved v1 library migrates and retains unreadable entries',()=>{
    const old=[legacyDesign(),{version:999}],raw=JSON.stringify(old);
    const storage={getItem:key=>key===LEGACY_STORAGE?raw:null};
    const r=readLibrary(storage);near(r.migrated,1);near(r.saved.length,1);near(r.rejected.length,1);
    assert(storage.getItem(LEGACY_STORAGE)===raw,'legacy storage changed');
    const current=readLibrary({getItem:key=>key===STORAGE?'[]':raw});near(current.saved.length,0);near(current.migrated,0);
  });
  return results;
}
