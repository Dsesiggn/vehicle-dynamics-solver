import test from 'node:test';
import assert from 'node:assert/strict';
import { newDesign, axle, clone } from '../src/model.js';
import { newAntiRollBar } from '../src/arb.js';
import { solveAxle, rotate } from '../src/solver.js';
import { SuspensionViewer, suspensionPreviewGeometry } from '../src/viewer.js';

const near=(actual,expected,tolerance=1e-9)=>assert.ok(Math.abs(actual-expected)<=tolerance,`${actual} != ${expected}`);
const vectorNear=(actual,expected)=>actual.forEach((value,index)=>near(value,expected[index]));
const context=(design,activeAxle)=>({activeAxle,wheelbase:design.wheelbase,obstacles:design.obstacles,linkOD:null});
const freeze=value=>{if(value&&typeof value==='object'){Object.freeze(value);Object.values(value).forEach(freeze);}return value;};

test('every topology assembles selected rear hardpoints one wheelbase behind front without moving the stored coordinates',()=>{
  const design=newDesign();design.wheelbase=1530;
  for(const topology of ['double-wishbone','macpherson','multi-link']) {
    const local=axle(topology);
    // A user-entered X offset is preserved relative to the rear datum.
    local.hardpoints.wheel_center[0]=24.125;
    const reference=clone(local);freeze(local);
    const front=suspensionPreviewGeometry(local,460,null,context(design,'front'));
    const rear=suspensionPreviewGeometry(local,460,null,context(design,'rear'));
    assert.deepEqual(rear.datums,{front:[0,0,0],rear:[-1530,0,0]});
    for(const corner of rear.corners) {
      for(const [key,p] of Object.entries(corner.points)) {
        const original=reference.hardpoints[key];
        vectorNear(p,[original[0]-1530,corner.side===-1?original[1]:-original[1],original[2]]);
        vectorNear(front.corners.find(item=>item.side===corner.side).points[key],[p[0]+1530,p[1],p[2]]);
      }
    }
    vectorNear(rear.corners[0].points.wheel_center,[-1505.875,reference.hardpoints.wheel_center[1],-230]);
    assert.deepEqual(local,reference);
    for(const segment of rear.chassis) near(segment[0][0],segment[1][0]);
  }
});

test('solved points and tire bounds translate once while wheel-axis directions and rigid distances are unchanged',()=>{
  const design=newDesign();design.wheelbase=1530;
  const local=design.axles.front;local.tireTreadWidth=203.2;
  const solved=solveAxle(local,15,8);assert.equal(solved.ok,true);
  const before=clone({local,solved});freeze(local);freeze(solved);
  const front=suspensionPreviewGeometry(local,460,solved,context(design,'front'));
  const rear=suspensionPreviewGeometry(local,460,solved,context(design,'rear'));
  for(let index=0;index<2;index++) {
    const original=index===0?solved.left:solved.right,a=front.corners[index],b=rear.corners[index];
    vectorNear(b.wheelAxis,rotate([0,b.side,0],original.rotation));
    assert.deepEqual(b.wheelAxis,a.wheelAxis);
    for(const key of Object.keys(original.points))vectorNear(b.points[key],[original.points[key][0]-1530,original.points[key][1],original.points[key][2]]);
    const length=points=>Math.hypot(...points.tie_rod_outer.map((value,i)=>value-points.tie_rod_inner[i]));
    near(length(b.points),length(original.points));
    vectorNear(b.tire.center,[a.tire.center[0]-1530,...a.tire.center.slice(1)]);
    for(const key of ['min','max'])vectorNear(b.tire.bounds[key],[a.tire.bounds[key][0]-1530,...a.tire.bounds[key].slice(1)]);
    b.tire.edgeCenters.forEach((point,i)=>vectorNear(point,[a.tire.edgeCenters[i][0]-1530,...a.tire.edgeCenters[i].slice(1)]));
  }
  assert.deepEqual({local,solved},before);
});

test('partial static ARB geometry uses the same rear translation at static and solved positions',()=>{
  const design=newDesign();design.wheelbase=1530;
  const local=design.axles.front;local.antiRollBar=newAntiRollBar();local.antiRollBar.enabled=true;
  Object.assign(local.antiRollBar.points,{bend:[-40,-200,-350],arm_tip:[80,-200,-350],link_pickup:[75,-400,-150]});
  const staticScene=suspensionPreviewGeometry(local,460,null,context(design,'rear'));
  const solved=solveAxle(local,15,0);assert.equal(solved.ok,true);
  const movingScene=suspensionPreviewGeometry(local,460,solved,context(design,'rear'));
  assert.deepEqual(movingScene.arb,staticScene.arb);
  assert.equal(staticScene.arb.points.length,6);
  assert.equal(staticScene.arb.points.some(point=>point.key==='bearing'),false);
  vectorNear(staticScene.arb.points.find(point=>point.key==='bend'&&point.side==='Left').position,[-1570,-200,-350]);
  vectorNear(staticScene.arb.points.find(point=>point.key==='bend'&&point.side==='Right').position,[-1570,200,-350]);
  for(const segment of staticScene.arb.segments) {
    assert.ok(staticScene.arb.points.some(point=>JSON.stringify(point.position)===JSON.stringify(segment.start)));
    assert.ok(staticScene.arb.points.some(point=>JSON.stringify(point.position)===JSON.stringify(segment.end)));
  }
});

test('shared packaging obstacles retain front-frame positions when switching active axles',()=>{
  const design=newDesign();design.wheelbase=1530;
  Object.assign(design.obstacles.engine,{visible:true,referenceAxle:'front',center:[120,30,-250],dimensions:{length:400,width:500,height:300}});
  Object.assign(design.obstacles.differential,{visible:true,referenceAxle:'rear',center:[130,-20,-180],dimensions:{length:100,width:120,height:140}});
  const before=clone(design);freeze(design);
  const front=suspensionPreviewGeometry(design.axles.front,460,null,context(design,'front'));
  const rear=suspensionPreviewGeometry(design.axles.rear,460,null,context(design,'rear'));
  assert.deepEqual(rear.obstacles,front.obstacles);
  for(const [key,expected] of [['engine',[120,30,-250]],['differential',[-1400,-20,-180]]]) {
    const mesh=rear.obstacles.find(item=>item.key===key).mesh;
    vectorNear([0,1,2].map(axis=>mesh.vertices.reduce((sum,p)=>sum+p[axis],0)/mesh.vertices.length),expected);
  }
  assert.deepEqual(design,before);
  assert.deepEqual(JSON.parse(JSON.stringify(design)),before);
});

test('the canvas labels both frame datums, fits named views, and refits wheelbase or axle changes',()=>{
  const saved={document:globalThis.document,window:globalThis.window,ResizeObserver:globalThis.ResizeObserver};
  const labels=[],coordinates=[];
  const canvasContext={beginPath(){},stroke(){},fill(){},closePath(){},setTransform(){},clearRect(){},setLineDash(){},
    moveTo(x,y){coordinates.push([x,y]);},lineTo(x,y){coordinates.push([x,y]);},arc(x,y){coordinates.push([x,y]);},
    fillText(text,x,y){labels.push({text,x,y});},measureText(text){return {width:text.length*5};}};
  const canvas={getContext(){return canvasContext;},getBoundingClientRect(){return {width:1000,height:650};},addEventListener(){}};
  try {
    globalThis.document={documentElement:{dataset:{theme:'light'}}};globalThis.window={devicePixelRatio:1};globalThis.ResizeObserver=class{observe(){}};
    const design=newDesign(),viewer=new SuspensionViewer(canvas);
    viewer.update(design.axles.rear,460,null,context(design,'rear'));
    assert.ok(labels.some(item=>item.text==='Front datum [0, 0, 0]'));
    assert.ok(labels.some(item=>item.text==='Rear datum [−1600, 0, 0]'));
    assert.ok(labels.some(item=>item.text==='Wheelbase 1600 mm'));
    for(const view of ['iso','top','front']) {
      labels.length=0;viewer.view(view);
      const datumLabels=labels.filter(item=>item.text.includes('datum'));
      assert.equal(datumLabels.length,2);
      assert.ok(datumLabels.every(item=>item.x>=0&&item.x<1000&&item.y>=0&&item.y<650));
      if(view==='front')assert.ok(labels.some(item=>item.text==='Wheelbase 1600 mm · along view'));
    }
    viewer.fitMode=false;viewer.zoom=2;design.wheelbase=1750;
    viewer.update(design.axles.rear,460,null,context(design,'rear'));
    assert.equal(viewer.fitMode,true);assert.equal(viewer.zoom,1);
    assert.ok(labels.some(item=>item.text==='Rear datum [−1750, 0, 0]'));
    viewer.fitMode=false;viewer.zoom=2;
    // Identical geometry must still refit when its reference axle changes.
    viewer.update(design.axles.rear,460,null,context(design,'front'));
    assert.equal(viewer.fitMode,true);assert.equal(viewer.zoom,1);
    assert.ok(coordinates.every(point=>point.every(Number.isFinite)));
  } finally {
    for(const [key,value] of Object.entries(saved))if(value===undefined)delete globalThis[key];else globalThis[key]=value;
  }
});
