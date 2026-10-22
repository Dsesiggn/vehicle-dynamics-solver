import test from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {exampleSteeringDesign,sweepSteering} from '../src/steering.js';
import {steeringConstruction} from '../src/steering-geometry.js';
import {requestSteeringSweep,validateSteeringSweepResponse,createSteeringSweepLoader} from '../src/steering-api.js';

// Reference-generated values exercise transport shape only, not Python physics.
function fixture() {
  const design=exampleSteeringDesign();design.steeringLimitDeg=5;
  const data={model:'fixed_axis_steering_v1',coordinate_system:'SAE_J670_Z_DOWN',hardpoint_frame:'AXLE_LOCAL',length_unit:'mm',angle_unit:'deg',step_deg:5,
    rows:sweepSteering(design).map(row=>({...row,construction:steeringConstruction(design,row)}))};
  return {design,data};
}
const response=(data,{status=200,contentType='application/json'}={})=>({ok:status>=200&&status<300,status,headers:{get:()=>contentType},json:async()=>data});
function deferred(){let resolve,reject;const promise=new Promise((yes,no)=>{resolve=yes;reject=no;});return{promise,resolve,reject};}

test('steering transport sends one same-origin-shaped POST and retains the validated Python construction',async()=>{
  const {design,data}=fixture();let captured;
  const result=await requestSteeringSweep(design,{endpoint:'http://127.0.0.1:8767/api/v1/steering/sweep',fetchImpl:async(url,options)=>{captured={url,options};return response(data);}});
  assert.equal(captured.url.pathname,'/api/v1/steering/sweep');
  assert.equal(captured.options.method,'POST');assert.equal(captured.options.credentials,'same-origin');assert.equal(captured.options.cache,'no-store');
  assert.deepEqual(JSON.parse(captured.options.body),{design});assert.strictEqual(result,data);assert.strictEqual(result.rows[2].construction,data.rows[2].construction);
});

test('wrong models, units, frame, sample order and extent are rejected before a sweep can be displayed',()=>{
  const {design,data}=fixture();
  const mutations=[d=>{d.model='different';},d=>{d.coordinate_system='Z_UP';},d=>{d.hardpoint_frame='CG';},d=>{d.length_unit='m';},d=>{d.angle_unit='rad';},d=>{d.step_deg=10;},d=>d.rows.pop(),d=>d.rows.reverse(),d=>{d.rows[1].steeringWheelDeg=5;}];
  for(const mutate of mutations){const invalid=structuredClone(data);mutate(invalid);assert.throws(()=>validateSteeringSweepResponse(invalid,design));}
});

test('nonfinite wheel, metric, construction and intersection values cannot enter the UI',()=>{
  const {design,data}=fixture();
  const mutations=[d=>{d.rows[0].left.ok=false;},d=>{d.rows[0].left.headingDeg=Infinity;},d=>{d.rows[0].left.outer[0]=NaN;},d=>{d.rows[0].metrics.totalToeInDeg=null;},d=>{d.rows[0].construction.left.planAngleDeg=Infinity;},d=>{d.rows[0].construction.left.planAxisPoint=undefined;},d=>{d.rows[0].construction.intersection.point[0]=Infinity;},d=>{d.rows[0].construction.pose='static';}];
  for(const mutate of mutations){const invalid=structuredClone(data);mutate(invalid);assert.throws(()=>validateSteeringSweepResponse(invalid,design));}
});

test('failed poses require a reason, cleared solved fields, and explicitly static construction',()=>{
  const {design,data}=fixture();
  data.rows[0]={ok:false,reason:'Entered rack stop exceeded.',steeringWheelDeg:-5,rack:-1,left:null,right:null,metrics:null,construction:steeringConstruction(design)};
  assert.strictEqual(validateSteeringSweepResponse(data,design),data);
  for(const mutate of [d=>{d.rows[0].reason='';},d=>{d.rows[0].left={headingDeg:1};},d=>{d.rows[0].construction.pose='solved';}]){const invalid=structuredClone(data);mutate(invalid);assert.throws(()=>validateSteeringSweepResponse(invalid,design));}
});

test('static servers and HTML responses give explicit engine-unavailable errors, not a browser fallback',async()=>{
  const {design}=fixture();
  for(const status of [404,405,501])await assert.rejects(requestSteeringSweep(design,{endpoint:'http://127.0.0.1:8766/api/v1/steering/sweep',fetchImpl:async()=>response(null,{status})}),{code:'ENGINE_UNAVAILABLE'});
  await assert.rejects(requestSteeringSweep(design,{endpoint:'https://example.test/api/v1/steering/sweep',fetchImpl:async()=>response(null,{contentType:'text/html'})}),{code:'ENGINE_UNAVAILABLE'});
  const app=readFileSync(new URL('../src/steering-app.js',import.meta.url),'utf8');
  assert.doesNotMatch(app,/\b(?:solveSteering|sweepSteering|steeringConstruction)\s*\(/);
});

test('HTTP rejection, unreadable JSON, lost connection and timeout fail explicitly',async()=>{
  const {design}=fixture(),endpoint='https://example.test/api/v1/steering/sweep';
  await assert.rejects(requestSteeringSweep(design,{endpoint,fetchImpl:async()=>response(null,{status:422})}),{code:'INPUT_REJECTED'});
  await assert.rejects(requestSteeringSweep(design,{endpoint,fetchImpl:async()=>response(null,{status:500})}),{code:'ENGINE_ERROR'});
  await assert.rejects(requestSteeringSweep(design,{endpoint,fetchImpl:async()=>({...response(null),json:async()=>{throw new Error('bad JSON');}})}),{code:'INVALID_RESPONSE'});
  await assert.rejects(requestSteeringSweep(design,{endpoint,fetchImpl:async()=>{throw new TypeError('network');}}),{code:'CONNECTION_FAILED'});
  await assert.rejects(requestSteeringSweep(design,{endpoint,timeoutMs:5,fetchImpl:(_url,{signal})=>new Promise((_resolve,reject)=>signal.addEventListener('abort',()=>reject(new Error('aborted')),{once:true}))}),{code:'ENGINE_TIMEOUT'});
});

test('a superseded request cannot overwrite the latest response even when the server ignores cancellation',async()=>{
  const {design,data}=fixture(),first=deferred(),second=deferred(),states=[],requests=[];
  const loader=createSteeringSweepLoader({debounceMs:0,request:(snapshot,options)=>{requests.push({snapshot,signal:options.signal});return requests.length===1?first.promise:second.promise;},onState:state=>states.push(state)});
  const old=loader.load(design);design.frontTrack=1300;const latest=loader.load(design);
  assert.equal(requests[0].snapshot.frontTrack,1200);assert.equal(requests[1].snapshot.frontTrack,1300);assert.equal(requests[0].signal.aborted,true);
  const fresh={...data,rows:[{marker:'latest'}]};second.resolve(fresh);await latest;first.resolve(data);assert.equal((await old).status,'superseded');
  assert.deepEqual(states.map(state=>state.status),['loading','loading','ready']);assert.strictEqual(states.at(-1).rows,fresh.rows);
});

test('clearing geometry cancels work and prevents a late success from restoring stale results',async()=>{
  const {design,data}=fixture(),pending=deferred(),states=[];
  const loader=createSteeringSweepLoader({debounceMs:0,request:()=>pending.promise,onState:state=>states.push(state)});
  const request=loader.load(design);loader.cancel();pending.resolve(data);assert.equal((await request).status,'superseded');
  assert.deepEqual(states.map(state=>state.status),['loading','idle']);assert.deepEqual(states.at(-1).rows,[]);
});

test('a new request clears prior results immediately and a subsequent failure keeps them cleared',async()=>{
  const {design,data}=fixture(),failed=deferred(),states=[];let count=0;
  const loader=createSteeringSweepLoader({debounceMs:0,request:()=>++count===1?Promise.resolve(data):failed.promise,onState:state=>states.push(state)});
  await loader.load(design);const next=loader.load(design);
  assert.equal(states.at(-1).status,'loading');assert.deepEqual(states.at(-1).rows,[]);
  failed.reject(new Error('offline'));await next;
  assert.equal(states.at(-1).status,'error');assert.deepEqual(states.at(-1).rows,[]);assert.equal(states.at(-1).error.message,'offline');
});

test('typing batches into one sweep while loading clears results immediately',async()=>{
  const {design,data}=fixture(),states=[],requests=[];
  const loader=createSteeringSweepLoader({debounceMs:15,request:async snapshot=>{requests.push(snapshot);return data;},onState:state=>states.push(state)});
  const first=loader.load(design);design.frontTrack=1300;const last=loader.load(design);
  assert.equal(requests.length,0);assert.equal(states.at(-1).status,'loading');assert.deepEqual(states.at(-1).rows,[]);
  assert.equal((await first).status,'superseded');assert.equal((await last).status,'ready');
  assert.equal(requests.length,1);assert.equal(requests[0].frontTrack,1300);
});

test('an incomplete edit cancels a queued sweep before any request is sent',async()=>{
  const {design,data}=fixture(),states=[];let requests=0;
  const loader=createSteeringSweepLoader({debounceMs:15,request:async()=>{requests++;return data;},onState:state=>states.push(state)});
  const pending=loader.load(design);loader.cancel();
  assert.equal((await pending).status,'superseded');assert.equal(requests,0);assert.equal(states.at(-1).status,'idle');
});
