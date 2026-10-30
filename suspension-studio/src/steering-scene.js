/** Display geometry only. Solved positions, axes and constructions remain SAE mm.
 * Unit rings and arcs are orientation glyphs, not tire/part dimensions.
 * See docs/COORDINATES.md and docs/STEERING.md; no steering closure is solved here.
 */
import {steeringConstruction} from './steering-geometry.js';

const finite=Number.isFinite;
export const isScenePoint=p=>Array.isArray(p)&&p.length===3&&[0,1,2].every(i=>finite(p[i]));
const add=(a,b)=>a.map((v,i)=>v+b[i]),sub=(a,b)=>a.map((v,i)=>v-b[i]),mul=(a,s)=>a.map(v=>v*s);
const dot=(a,b)=>a.reduce((sum,v,i)=>sum+v*b[i],0),norm=a=>Math.hypot(...a);
const cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
const unit=a=>{const n=norm(a);return finite(n)&&n>0?mul(a,1/n):null;};
const ground=p=>[p[0],p[1],0];
export const saeToThree=([x,y,z])=>[y,-z,-x];

/** Unit-radius, axisymmetric orientation ring; radius is a renderer glyph size. */
export function wheelRingOffsets(normal,segments=64){
  if(!isScenePoint(normal)||!Number.isInteger(segments)||segments<8)return [];
  const n=unit(normal);if(!n)return [];
  const least=n.map(Math.abs).indexOf(Math.min(...n.map(Math.abs))),reference=[0,0,0];reference[least]=1;
  const u=unit(cross(n,reference)),v=cross(n,u);
  return Array.from({length:segments},(_,i)=>add(mul(u,Math.cos(2*Math.PI*i/segments)),mul(v,Math.sin(2*Math.PI*i/segments))));
}

/** Minor included-angle glyph in the actual plane of two rays. Collinear rays
 * have no unique arc plane, so no arbitrary semicircle is invented at 180°.
 */
export function includedAngleArcOffsets(rayA,rayB,segments=32){
  if(!isScenePoint(rayA)||!isScenePoint(rayB)||!Number.isInteger(segments)||segments<2)return [];
  const a=unit(rayA),b=unit(rayB);if(!a||!b)return [];
  const perpendicular=cross(a,b),sine=norm(perpendicular);if(sine<1e-12)return [];
  const axis=mul(perpendicular,1/sine),tangent=cross(axis,a),angle=Math.atan2(sine,dot(a,b));
  return Array.from({length:segments+1},(_,i)=>add(mul(a,Math.cos(angle*i/segments)),mul(tangent,Math.sin(angle*i/segments))));
}

function staticSpindle(side,geometry){
  const s=side==='left'?-1:1,h=-s*geometry.staticToeDeg*Math.PI/180,c=geometry.staticCamberDeg*Math.PI/180;
  return [-s*Math.sin(h)*Math.cos(c),s*Math.cos(h)*Math.cos(c),Math.sin(c)];
}
function clippedPlanLine(origin,other,bounds,segmentOnly=false){
  const direction=sub(other,origin);if(Math.hypot(direction[0],direction[1])===0)return null;
  let low=segmentOnly?0:-Infinity,high=segmentOnly?1:Infinity;
  for(let i=0;i<2;i++){
    if(direction[i]===0){if(origin[i]<bounds.min[i]||origin[i]>bounds.max[i])return null;continue;}
    const a=(bounds.min[i]-origin[i])/direction[i],b=(bounds.max[i]-origin[i])/direction[i];
    low=Math.max(low,Math.min(a,b));high=Math.min(high,Math.max(a,b));
  }
  if(low>high||!finite(low)||!finite(high))return null;
  return [ground(add(origin,mul(direction,low))),ground(add(origin,mul(direction,high)))];
}
export function sceneBounds(points){
  const valid=points.filter(isScenePoint);if(!valid.length)return null;
  return {min:[0,1,2].map(i=>Math.min(...valid.map(p=>p[i]))),max:[0,1,2].map(i=>Math.max(...valid.map(p=>p[i])))};
}

export function buildSteeringScene(design,result,{ready=true,frontOnly=true,showIdeal=true,showArmLines=true,showArmAngles=true,fitIntersection=false}={}){
  const scene={pose:'empty',segments:[],markers:[],rings:[],arcs:[],headings:[],labels:[],boundsPoints:[],intersection:null};
  if(!ready||!design)return scene;
  const goodPoint=isScenePoint;
  const segment=(id,points,style,side=null,bounded=true)=>{if(points.every(goodPoint)){const item={id,points:points.map(p=>[...p]),style,side};scene.segments.push(item);if(bounded)scene.boundsPoints.push(...item.points);}};
  const marker=(id,position,style,side=null,bounded=true)=>{if(goodPoint(position)){scene.markers.push({id,position:[...position],style,side});if(bounded)scene.boundsPoints.push([...position]);}};
  const label=(text,position,style='reference',side=null)=>{if(goodPoint(position))scene.labels.push({text,position:[...position],style,side});};
  const ring=(id,center,normal,style,side)=>{if(!goodPoint(center)||!goodPoint(normal))return;const offsets=wheelRingOffsets(normal);if(offsets.length){scene.rings.push({id,center:[...center],normal:[...normal],unitOffsets:offsets,style,side,radiusPx:27});scene.boundsPoints.push([...center]);}};
  const arc=(id,center,a,b,value,style,side)=>{if(![center,a,b].every(goodPoint)||!finite(value))return;const rayA=sub(a,center),rayB=sub(b,center),offsets=includedAngleArcOffsets(rayA,rayB);if(offsets.length)scene.arcs.push({id,center:[...center],unitOffsets:offsets,maxRadiusMm:Math.min(norm(rayA),norm(rayB))*.35,style,side,angleDeg:value,label:`${style==='projection'?'α':'β'}${side==='left'?'L':'R'} ${value.toFixed(1)}°`});};
  const solved=result?.ok===true,neutral=steeringConstruction(design),construction=result?.construction||(solved?null:neutral);
  scene.pose=solved?'solved':'static';scene.intersection=construction?.intersection||null;
  const centers={};
  for(const side of ['left','right']){
    const a=design[side];if(!a)continue;
    const s=side==='left'?-1:1,staticCenter=[0,s*design.frontTrack/2,a.wheelCenterZ],current=solved?result[side]:null,part=construction?.[side],original=neutral[side];
    const outer=current?.outer||a.tieOuter,inner=current?.inner||a.tieInner,center=current?.wheelCenter||staticCenter,normal=current?.spindle||staticSpindle(side,a);
    centers[side]={inner,center};
    segment(`${side}:steering-axis`,[a.axisUpper,a.axisLower],'actual',side);
    segment(`${side}:tie-rod`,[inner,outer],'actual',side);
    segment(`${side}:effective-arm`,[part?.axisFoot,outer],'radius',side);
    segment(`${side}:static-tie`,[a.tieInner,a.tieOuter],'static',side);
    segment(`${side}:static-arm`,[original?.axisFoot,a.tieOuter],'static',side);
    for(const [name,point] of [['U',a.axisUpper],['B',a.axisLower],['O',outer],['I',inner],['C',part?.axisFoot]])marker(`${side}:${name}`,point,'actual',side);
    marker(`${side}:wheel-center`,center,'actual',side);
    ring(`${side}:wheel-plane`,center,normal,'actual',side);
    ring(`${side}:static-wheel-plane`,staticCenter,staticSpindle(side,a),'static',side);
    label(`${side==='left'?'L':'R'} wheel plane`,center,'actual',side);
    label(`${side==='left'?'L':'R'} U`,a.axisUpper,'actual',side);
    if(showArmAngles)arc(`${side}:beta`,outer,part?.axisFoot,inner,part?.spatialAngleDeg,'actual',side);
    if(showArmLines||showArmAngles){
      // K can be arbitrarily far away for a nearly horizontal axis. It is a
      // construction point, so it must never dominate the normal physical fit.
      if(goodPoint(outer))segment(`${side}:drop-outer`,[outer,ground(outer)],'drop',side);
      if(goodPoint(outer)&&goodPoint(part?.planAxisPoint)){
        if(goodPoint(inner))segment(`${side}:plan-tie`,[ground(inner),ground(outer)],'projection',side);
        if(showArmAngles&&goodPoint(inner))arc(`${side}:alpha`,ground(outer),ground(part.planAxisPoint),ground(inner),part.planAngleDeg,'projection',side);
      }
    }
  }
  segment('rack:current',[centers.left?.inner,centers.right?.inner],'rack');
  segment('rack:static',[design.left?.tieInner,design.right?.tieInner],'static');
  if(finite(design.frontTrack))segment('front-axle:datum',[[0,-design.frontTrack/2,0],[0,design.frontTrack/2,0]],'reference');
  marker('front-datum',[0,0,0],'reference');label('Front axle datum · Z = 0',[0,0,0]);
  if(!frontOnly&&finite(design.wheelbase)&&finite(design.rearTrack)){
    const rear=[-design.wheelbase,0,0];segment('rear-axle:reference',[[-design.wheelbase,-design.rearTrack/2,0],[-design.wheelbase,design.rearTrack/2,0]],'reference');
    segment('vehicle:center-datum',[[0,0,0],rear],'reference');marker('rear-left:reference',[-design.wheelbase,-design.rearTrack/2,0],'reference');marker('rear-right:reference',[-design.wheelbase,design.rearTrack/2,0],'reference');label('Rear axle reference · Z = 0',rear);
  }
  const intersection=construction?.intersection;
  if(showArmLines&&intersection?.status==='finite'&&goodPoint(intersection.point)){
    marker('arm-intersection:plan',ground(intersection.point),'projection',null,fitIntersection);
    label('Arm intersection · plan projection Z = 0',ground(intersection.point),'projection');
  }
  const bounds=sceneBounds(scene.boundsPoints);
  if((showArmLines||showArmAngles)&&bounds){
    const padding=Math.max(1,bounds.max[0]-bounds.min[0],bounds.max[1]-bounds.min[1])*.12;
    const clip={min:bounds.min.map(x=>x-padding),max:bounds.max.map(x=>x+padding)};
    for(const side of ['left','right']){
      const part=construction?.[side];if(!goodPoint(part?.planAxisPoint)||!goodPoint(part?.outer))continue;
      const k=part.planAxisPoint;
      if(k[0]>=clip.min[0]&&k[0]<=clip.max[0]&&k[1]>=clip.min[1]&&k[1]<=clip.max[1])segment(`${side}:drop-axis-height`,[k,ground(k)],'drop',side,false);
      const arm=clippedPlanLine(ground(k),ground(part.outer),clip,true);if(arm)segment(`${side}:plan-arm`,arm,'projection',side,false);
      if(showArmLines){const line=clippedPlanLine(ground(k),ground(part.outer),clip);if(line)segment(`${side}:arm-extension:plan`,line,'projection',side,false);}
    }
  }
  const metrics=solved?result.metrics:null;
  if(showIdeal&&metrics?.insideSide&&finite(metrics.idealOutsideDeg))for(const side of ['left','right']){
    const heading=(metrics.turn==='right'?1:-1)*(side===metrics.insideSide?metrics.insideDeg:metrics.idealOutsideDeg)*Math.PI/180;
    if(finite(heading))scene.headings.push({id:`${side}:ideal-heading:plan`,center:[0,(side==='left'?-1:1)*design.frontTrack/2,0],direction:[Math.cos(heading),Math.sin(heading),0],style:'ideal',side,lengthPx:36});
  }
  return scene;
}
