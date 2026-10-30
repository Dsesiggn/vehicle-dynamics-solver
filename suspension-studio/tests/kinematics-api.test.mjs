import test from 'node:test';
import assert from 'node:assert/strict';
import {newDesign} from '../src/model.js';
import {KinematicsApiError,validateKinematicsSweepResponse,requestKinematicsSweep,createKinematicsSweepLoader} from '../src/kinematics-api.js';

const endpoint='http://127.0.0.1:8768/api/v1/kinematics/sweep';
function fixture(){
  const request={design:newDesign(),sweep:{mode:'heave',min:-1,max:1,samples:3}};
  const corner=(axle,side)=>({ok:true,error:0,points:structuredClone(axle.hardpoints),rotation:[0,0,0],spindle:[0,side==='left'?-1:1,0],camber:0,toe:0,steer:0,compression:0,actuator:{ok:true},alignmentReference:'entered-static-alignment'});
  const data={model:'rigid_suspension_v1',coordinate_system:'SAE_J670_Z_DOWN',hardpoint_frame:'AXLE_LOCAL',length_unit:'mm',angle_unit:'deg',mode:'heave',input_unit:'mm',
    channels:[{id:'camber.front.left',plot:'camber',title:'Camber',label:'Front left',unit:'deg',group:'Alignment',entity:'front.left',description:'Chassis-relative camber.'},{id:'camber.front.right',plot:'camber',title:'Camber',label:'Front right',unit:'deg',group:'Alignment',entity:'front.right',description:'Chassis-relative camber.'}],
    rows:[-1,0,1].map(input=>({input,values:{'camber.front.left':0,'camber.front.right':null},unavailable:{'camber.front.right':'Static alignment is unentered.'},poses:Object.fromEntries(Object.entries(request.design.axles).map(([name,axle])=>[name,{ok:true,left:corner(axle,'left'),right:corner(axle,'right')}])),diagnostics:{maxResidualMm:0,maxConditionNumber:40}})),
    assumptions:['Rigid links.'],warnings:[],coverage:[{family:'Camber',status:'available',reason:'Requires entered static alignment.'}]};
  return {request,data};
}
const response=(data,status=200,type='application/json')=>({ok:status>=200&&status<300,status,headers:{get:()=>type},json:async()=>data});
const rejectsCode=(promise,code)=>assert.rejects(promise,error=>error instanceof KinematicsApiError&&error.code===code);

test('valid responses retain zero, unavailable reasons, geometry and channel metadata without mutation',()=>{
  const {request,data}=fixture(),before=JSON.stringify(data);
  assert.equal(validateKinematicsSweepResponse(data,request),data);
  assert.equal(JSON.stringify(data),before);
  assert.equal(data.rows[0].values['camber.front.left'],0);
  assert.equal(data.rows[0].values['camber.front.right'],null);
});

test('coordinate, model, mode, units and sample ordering mismatches are rejected',()=>{
  const mutations=[
    data=>data.model='unverified_v1',data=>data.coordinate_system='Z_UP',data=>data.hardpoint_frame='FRONT_AXLE',data=>data.length_unit='in',
    data=>data.mode='roll',data=>data.input_unit='deg',data=>data.rows.reverse(),data=>data.rows.pop(),data=>data.rows[1].input=0.01,
  ];
  for(const mutate of mutations){const {request,data}=fixture();mutate(data);assert.throws(()=>validateKinematicsSweepResponse(data,request),KinematicsApiError);}
});

test('nonfinite and missing channel values cannot masquerade as available data',()=>{
  const mutations=[
    data=>data.rows[0].values['camber.front.left']=NaN,data=>data.rows[0].values['camber.front.left']=Infinity,
    data=>delete data.rows[0].values['camber.front.left'],data=>data.rows[0].values['camber.front.left']='0',
    data=>delete data.rows[0].unavailable['camber.front.right'],data=>data.rows[0].unavailable['camber.front.right']='',
    data=>data.rows[0].unavailable['camber.front.left']='Contradictory reason',data=>data.rows[0].values.extra=1,
  ];
  for(const mutate of mutations){const {request,data}=fixture();mutate(data);assert.throws(()=>validateKinematicsSweepResponse(data,request),KinematicsApiError);}
});

test('duplicate channels, mixed plot units and malformed coverage are rejected',()=>{
  const mutations=[
    data=>data.channels[1].id=data.channels[0].id,data=>data.channels[1].unit='mm',data=>data.channels[1].title='Other metric',
    data=>data.channels[0].entity='unknown',data=>data.channels[0].description='',data=>data.coverage[0].status='assumed',
    data=>data.assumptions=null,data=>data.warnings=[null],
  ];
  for(const mutate of mutations){const {request,data}=fixture();mutate(data);assert.throws(()=>validateKinematicsSweepResponse(data,request),KinematicsApiError);}
});

test('malformed solved poses and contradictory actuator availability are rejected',()=>{
  const mutations=[
    pose=>pose.points.wheel_center=[0,Infinity,0],pose=>delete pose.points.wheel_center,pose=>pose.rotation=[0,0],
    pose=>pose.spindle=[0,0,0],pose=>pose.camber=undefined,pose=>pose.error=-1,
    pose=>{pose.actuator={ok:false,reason:'Axis unknown'};},pose=>{pose.actuator={ok:false};pose.compression=null;},
  ];
  for(const mutate of mutations){const {request,data}=fixture();mutate(data.rows[0].poses.front.left);assert.throws(()=>validateKinematicsSweepResponse(data,request),KinematicsApiError);}
  const {request,data}=fixture();
  data.rows[0].poses.front.left.actuator={ok:false,reason:'Axis unknown'};data.rows[0].poses.front.left.compression=null;
  assert.equal(validateKinematicsSweepResponse(data,request),data);
});

test('failed corners require a reason, contain no solved geometry and agree with axle status',()=>{
  const {request,data}=fixture();
  data.rows[0].poses.front.left={ok:false,reason:'Singular geometry',error:null};data.rows[0].poses.front.ok=false;
  assert.equal(validateKinematicsSweepResponse(data,request),data);
  data.rows[0].poses.front.ok=true;
  assert.throws(()=>validateKinematicsSweepResponse(data,request),KinematicsApiError);
  data.rows[0].poses.front.ok=false;data.rows[0].poses.front.left.points={};
  assert.throws(()=>validateKinematicsSweepResponse(data,request),KinematicsApiError);
});

test('request sends the exact design/sweep once to the supplied same-origin-style endpoint',async()=>{
  const {request,data}=fixture();let captured;
  const result=await requestKinematicsSweep(request,{endpoint,fetchImpl:async(url,options)=>{captured={url,options};return response(data);}});
  assert.equal(result,data);assert.equal(String(captured.url),endpoint);assert.equal(captured.options.method,'POST');
  assert.deepEqual(JSON.parse(captured.options.body),request);assert.equal(captured.options.credentials,'same-origin');assert.equal(captured.options.cache,'no-store');
});

test('file URLs, static servers, HTML pages, input rejection and connection errors are explicit',async()=>{
  const {request,data}=fixture();
  await rejectsCode(requestKinematicsSweep(request,{endpoint:'file:///api/v1/kinematics/sweep'}),'HTTP_REQUIRED');
  for(const status of [404,405,501])await rejectsCode(requestKinematicsSweep(request,{endpoint,fetchImpl:async()=>response(data,status)}),'ENGINE_UNAVAILABLE');
  await rejectsCode(requestKinematicsSweep(request,{endpoint,fetchImpl:async()=>response(data,200,'text/html')}),'ENGINE_UNAVAILABLE');
  await rejectsCode(requestKinematicsSweep(request,{endpoint,fetchImpl:async()=>response(data,422)}),'INPUT_REJECTED');
  await rejectsCode(requestKinematicsSweep(request,{endpoint,fetchImpl:async()=>response(data,500)}),'ENGINE_ERROR');
  await rejectsCode(requestKinematicsSweep(request,{endpoint,fetchImpl:async()=>{throw new TypeError('offline');}}),'CONNECTION_FAILED');
  await rejectsCode(requestKinematicsSweep(request,{endpoint,fetchImpl:async()=>({...response(data),json:async()=>{throw new Error('invalid json');}})}),'INVALID_RESPONSE');
});

test('timeout aborts the transport and does not return partial results',async()=>{
  const {request}=fixture();let aborted=false;
  await rejectsCode(requestKinematicsSweep(request,{endpoint,timeoutMs:5,fetchImpl:async(_url,{signal})=>new Promise((_resolve,reject)=>signal.addEventListener('abort',()=>{aborted=true;reject(new DOMException('Aborted','AbortError'));},{once:true}))}),'ENGINE_TIMEOUT');
  assert.equal(aborted,true);
});

test('caller cancellation stays cancellation including after response parsing',async()=>{
  const {request,data}=fixture();const controller=new AbortController();
  await assert.rejects(requestKinematicsSweep(request,{endpoint,signal:controller.signal,fetchImpl:async()=>({...response(data),json:async()=>{controller.abort(new DOMException('Cancelled','AbortError'));return data;}})}),error=>error.name==='AbortError');
});

test('loader snapshots inputs and rejects a superseded response even if transport ignores abort',async()=>{
  const states=[],pending=[];
  const loader=createKinematicsSweepLoader({onState:state=>states.push(state),request:async(inputs,options)=>new Promise(resolve=>pending.push({inputs,options,resolve}))});
  const original={design:{name:'Original'},sweep:{mode:'heave'}};
  const first=loader.load(original);original.design.name='Edited';
  const second=loader.load(original);
  assert.equal(pending[0].inputs.design.name,'Original');assert.equal(pending[0].options.signal.aborted,true);
  pending[1].resolve({rows:[{input:2}]});assert.equal((await second).status,'ready');
  pending[0].resolve({rows:[{input:1}]});assert.equal((await first).status,'superseded');
  assert.equal(states.filter(state=>state.status==='ready').length,1);assert.equal(states.at(-1).rows[0].input,2);
});

test('loader clears prior results on error or cancel and cannot resurrect a cancelled result',async()=>{
  const states=[];let resolve;
  const loader=createKinematicsSweepLoader({onState:state=>states.push(state),request:()=>new Promise(done=>{resolve=done;})});
  const pending=loader.load({});loader.cancel();resolve({rows:[{input:1}]});
  assert.equal((await pending).status,'superseded');assert.deepEqual(states.at(-1),{status:'idle',rows:[]});
  const failing=createKinematicsSweepLoader({onState:state=>states.push(state),request:async()=>{throw new Error('Unavailable');}});
  assert.equal((await failing.load({})).status,'error');assert.deepEqual(states.at(-1).rows,[]);
});
