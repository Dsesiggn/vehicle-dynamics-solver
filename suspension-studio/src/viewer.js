import { mirrorPoint, toDisplay } from './coordinates.js';
import {linkPairs,isMoving,hardpointLabel} from './model.js?v=0.2.8';
import {add,sub,scale,unit,cross,rotate} from './solver.js?v=0.2.8';
import {arbPreviewGeometry} from './arb.js?v=0.2.7';
import {OBSTACLE_TYPES,obstacleMesh} from './obstacles.js?v=0.2.5';
import {memberSegmentId} from './interference.js?v=0.2.8';
import {tireTreadGeometry} from './tire-geometry.js?v=0.2.8';
const PALETTES={
  light:{arb:'#448272',arm:'#71889d',actuator:'#98627f',steering:'#ad976c',upright:'#8f86a1',point:'#a98dbc',chassis:'#b6b8c7',grid:'#bcc0cb',centerline:'#b0acbc',pointFill:'#faf8fc',label:'#766b83',tire:'#a0a2b6',rim:'#aaa4b7',axisX:'#ae8497',axisY:'#8c9b86',axisZ:'#8b93af',engine:'#b57a2a',differential:'#39769b',cockpit:'#80558e',interference:'#d21f2b'},
  dark:{arb:'#92d4bf',arm:'#a8c6de',actuator:'#df9fbe',steering:'#dec18b',upright:'#c4b3df',point:'#e2bdf3',chassis:'#8795a8',grid:'#55637a',centerline:'#8795a8',pointFill:'#1b2330',label:'#d7cbe3',tire:'#a4b1c8',rim:'#b5a9ce',axisX:'#efadbb',axisY:'#abcba3',axisZ:'#b3c9f3',engine:'#f0bd70',differential:'#82c8eb',cockpit:'#d5a6e7',interference:'#ff716c'},
};
export class SuspensionViewer {
  constructor(canvas) {
    this.canvas=canvas;this.ctx=canvas.getContext('2d');this.yaw=-.48;this.pitch=.32;this.zoom=1;this.labels=false;this.fitMode=false;this.visibleObstacleKeys=new Set();
    new ResizeObserver(()=>this.draw()).observe(canvas);
    canvas.addEventListener('pointerdown',e=>{this.drag=[e.clientX,e.clientY];canvas.setPointerCapture(e.pointerId);});
    canvas.addEventListener('pointermove',e=>{if(!this.drag)return;this.yaw+=(e.clientX-this.drag[0])*.007;this.pitch=Math.max(-1.5,Math.min(1.5,this.pitch+(e.clientY-this.drag[1])*.007));this.drag=[e.clientX,e.clientY];this.draw();});
    canvas.addEventListener('pointerup',()=>this.drag=null);canvas.addEventListener('pointercancel',()=>this.drag=null);
    canvas.addEventListener('wheel',e=>{e.preventDefault();this.zoom=Math.max(.5,Math.min(2.5,this.zoom*Math.exp(-e.deltaY*.001)));this.draw();},{passive:false});
    canvas.addEventListener('keydown',e=>{if(!['ArrowLeft','ArrowRight','ArrowUp','ArrowDown','+','-','='].includes(e.key))return;e.preventDefault();if(e.key==='ArrowLeft')this.yaw-=.12;if(e.key==='ArrowRight')this.yaw+=.12;if(e.key==='ArrowUp')this.pitch=Math.min(1.5,this.pitch+.1);if(e.key==='ArrowDown')this.pitch=Math.max(-1.5,this.pitch-.1);if(e.key==='+'||e.key==='=')this.zoom=Math.min(2.5,this.zoom*1.1);if(e.key==='-')this.zoom=Math.max(.5,this.zoom/1.1);this.draw();});
  }
  view(name) {this.yaw=name==='iso'?-.48:0;this.pitch=name==='top'?Math.PI/2:name==='front'?0:.32;this.zoom=1;this.fitMode=false;this.draw();}
  fitAll() {this.fitMode=true;this.zoom=1;this.draw();}
  update(a,diameter,motion=null,context={}) {
    this.axle=a;this.diameter=diameter;this.motion=motion;this.context=context;
    const current=new Set(OBSTACLE_TYPES.filter(({key})=>context.obstacles?.[key]?.visible&&obstacleMesh(context.obstacles[key],key,context.activeAxle,context.wheelbase)).map(({key})=>key));
    const arbPreview=arbPreviewGeometry(a.antiRollBar),arbSignature=JSON.stringify(arbPreview.points.map(({key,position})=>[key,position]));
    const treadWidth=a.tireTreadWidth??null,treadWidthChanged=treadWidth!==this.lastTireTreadWidth&&(treadWidth!==null||this.lastTireTreadWidth!=null);
    if(treadWidthChanged||[...current].some(key=>!this.visibleObstacleKeys.has(key))||arbPreview.points.length&&arbSignature!==this.lastArbSignature||this.lastAxle&&this.lastAxle!==context.activeAxle&&(current.size||arbPreview.points.length)){this.fitMode=true;this.zoom=1;}
    this.lastTireTreadWidth=treadWidth;
    this.lastArbSignature=arbSignature;
    this.lastAxle=context.activeAxle;
    this.visibleObstacleKeys=current;this.draw();
  }
  draw() {
    if(!this.axle)return;
    const colors=PALETTES[document.documentElement.dataset.theme]||PALETTES.light;
    const c=this.ctx,canvas=this.canvas,rect=canvas.getBoundingClientRect(),w=rect.width,h=rect.height,dpr=window.devicePixelRatio||1;
    canvas.width=Math.round(w*dpr);canvas.height=Math.round(h*dpr);c.setTransform(dpr,0,0,dpr,0,0);c.clearRect(0,0,w,h);
    const a=this.axle;
    const obstacleMeshes=OBSTACLE_TYPES.map(({key,label})=>({key,label,mesh:obstacleMesh(this.context?.obstacles?.[key],key,this.context?.activeAxle,this.context?.wheelbase)})).filter(item=>item.mesh);
    const obstacleVertices=obstacleMeshes.flatMap(item=>item.mesh.vertices);
    const localPoints=[];
    for(const side of [-1,1]) {
      const solved=this.motion?.ok?(side===-1?this.motion.left:this.motion.right):null;
      const points=solved?.points||Object.fromEntries(Object.entries(a.hardpoints).map(([key,value])=>[key,side===-1?value:mirrorPoint(value)]));
      localPoints.push(...Object.values(points));
      const tire=tireTreadGeometry(points.wheel_center,rotate([0,side,0],solved?.rotation||[0,0,0]),this.diameter,a.tireTreadWidth??null);
      if(tire)for(const x of [0,1])for(const y of [0,1])for(const z of [0,1])localPoints.push([x?tire.bounds.max[0]:tire.bounds.min[0],y?tire.bounds.max[1]:tire.bounds.min[1],z?tire.bounds.max[2]:tire.bounds.min[2]]);
    }
    const arbPreview=arbPreviewGeometry(a.antiRollBar);
    localPoints.push(...arbPreview.points.map(({position})=>position));
    const projectBasis=point=>{const p=toDisplay(point),x=p[0]*Math.cos(this.yaw)-p[1]*Math.sin(this.yaw),depth=p[0]*Math.sin(this.yaw)+p[1]*Math.cos(this.yaw);return [x,(p[2]-this.diameter*.3)*Math.cos(this.pitch)-depth*Math.sin(this.pitch),depth*Math.cos(this.pitch)+p[2]*Math.sin(this.pitch)];};
    let s=Math.min(w/(a.track+this.diameter*.5+180),h/(this.diameter+350))*this.zoom,centerX=0,centerVertical=0,screenCenterY=h*.64;
    if(this.fitMode) {
      const bounds=[...localPoints,...obstacleVertices].map(projectBasis),xs=bounds.map(p=>p[0]),ys=bounds.map(p=>p[1]);
      const minX=Math.min(...xs),maxX=Math.max(...xs),minY=Math.min(...ys),maxY=Math.max(...ys),spanX=Math.max(1,maxX-minX),spanY=Math.max(1,maxY-minY);
      s=Math.min(Math.max(80,w-64)/spanX,Math.max(80,h-112)/spanY)*this.zoom;
      centerX=(minX+maxX)/2;centerVertical=(minY+maxY)/2;screenCenterY=h*.54;
    }
    const project=point=>{const [x,vertical,depth]=projectBasis(point);return [w/2+(x-centerX)*s,screenCenterY-(vertical-centerVertical)*s,depth];};
    const drawLine=(points,color,width=1,alpha=1,dash=[])=>{c.beginPath();points.forEach((p,i)=>{const [x,y]=project(p);i?c.lineTo(x,y):c.moveTo(x,y);});c.strokeStyle=color;c.globalAlpha=alpha;c.lineWidth=width;c.setLineDash(dash);c.stroke();c.setLineDash([]);c.globalAlpha=1;};
    for(let x=-2000;x<=2000;x+=100)drawLine([[-900,x,0],[900,x,0]],colors.grid,.6,.3);
    for(let y=-900;y<=900;y+=100)drawLine([[y,-2000,0],[y,2000,0]],colors.grid,.6,.3);
    drawLine([[0,-1800,0],[0,1800,0]],colors.centerline,.8,.5);
    const objects=[];
    const line=(points,color,width=2,alpha=1)=>objects.push({depth:points.reduce((sum,p)=>sum+project(p)[2],0)/points.length,render:()=>drawLine(points,color,width,alpha)});
    const checkedColor=id=>this.context?.highlightSegments?.has(id)?colors.interference:null;
    const point=(p,label,color=colors.point,radius=2.6)=>objects.push({depth:project(p)[2]+1,render:()=>{const [x,y]=project(p);c.beginPath();c.arc(x,y,radius,0,Math.PI*2);c.fillStyle=colors.pointFill;c.fill();c.strokeStyle=color;c.lineWidth=1.2;c.stroke();if(this.labels){c.fillStyle=colors.label;c.font='7px system-ui';c.fillText(label,x+5,y-5);}}});
    const ring=(p,axis,radius)=>{
      const v=unit(cross(axis,Math.abs(axis[2])>.9?[1,0,0]:[0,0,1])),u=cross(axis,v);
      return Array.from({length:33},(_,i)=>add(p,add(scale(v,radius*Math.cos(i*Math.PI/16)),scale(u,radius*Math.sin(i*Math.PI/16)))));
    };
    const cylinder=(p1,p2,radius,color,knownAxis=null)=>{
      const axis=knownAxis||unit(sub(p2,p1)),v=unit(cross(axis,Math.abs(axis[2])>.9?[1,0,0]:[0,0,1])),u=cross(axis,v);
      line(ring(p1,axis,radius),color,1.2,.75);line(ring(p2,axis,radius),color,1.2,.75);
      for(let i=0;i<8;i++) {const offset=add(scale(v,radius*Math.cos(i*Math.PI/4)),scale(u,radius*Math.sin(i*Math.PI/4)));line([add(p1,offset),add(p2,offset)],color,.7,.35);}
    };
    for(const side of [-1,1]) {
      const solved=this.motion?.ok?(side===-1?this.motion.left:this.motion.right):null;
      const p=solved?.points||Object.fromEntries(Object.entries(a.hardpoints).map(([k,v])=>[k,side===-1?v:mirrorPoint(v)]));
      const sideKey=side===-1?'left':'right';
      for(const [i,o] of linkPairs(a)) {
        const id=memberSegmentId(sideKey,'link',i,o),color=checkedColor(id)||(i==='tie_rod_inner'?colors.steering:colors.arm);
        if(this.context?.linkOD>0)cylinder(p[i],p[o],this.context.linkOD/2,color);else line([p[i],p[o]],color,2.7);
      }
      const upright=Object.entries(p).filter(([k])=>isMoving(k)&&k!=='wheel_center'&&k!=='actuation_outer').map(([,v])=>v);
      for(const v of upright)line([v,p.wheel_center],colors.upright,1.4,.6);
      if(a.topology==='double-wishbone') {line([p.uca_front,p.uca_rear],colors.arm,1,.4);line([p.lca_front,p.lca_rear],colors.arm,1,.4);}
      let damperStart,damperEnd;
      let damperId;
      if(a.topology==='macpherson'){damperStart=p.strut_top;damperEnd=p.strut_bottom;damperId=memberSegmentId(sideKey,'damper','strut_top','strut_bottom');}
      else if(a.actuation==='direct'){damperStart=p.damper_top;damperEnd=p.actuation_outer;damperId=memberSegmentId(sideKey,'damper','damper_top','actuation_outer');}
      else {
        const rodId=memberSegmentId(sideKey,'link','actuation_outer','rocker_rod'),rodColor=checkedColor(rodId)||colors.actuator;
        if(this.context?.linkOD>0)cylinder(p.actuation_outer,p.rocker_rod,this.context.linkOD/2,rodColor);else line([p.actuation_outer,p.rocker_rod],rodColor,2.7);
        for(const [start,end] of [['rocker_pivot','rocker_rod'],['rocker_rod','rocker_damper'],['rocker_damper','rocker_pivot']]) {
          const id=memberSegmentId(sideKey,'link',start,end),color=checkedColor(id)||colors.actuator;
          if(this.context?.linkOD>0)cylinder(p[start],p[end],this.context.linkOD/2,color);else line([p[start],p[end]],color,2);
        }
        damperStart=p.damper_top;damperEnd=p.rocker_damper;
        damperId=memberSegmentId(sideKey,'damper','damper_top','rocker_damper');
      }
      cylinder(damperStart,damperEnd,a.damperOD/2,checkedColor(damperId)||colors.actuator);
      const axis=unit(sub(damperEnd,damperStart)),v=unit(cross(axis,Math.abs(axis[0])>.9?[0,1,0]:[1,0,0])),u=cross(axis,v);
      const helix=Array.from({length:161},(_,i)=>{const t=i/160;return add(add(damperStart,scale(sub(damperEnd,damperStart),.12+.7*t)),add(scale(v,a.springOD/2*Math.cos(t*Math.PI*16)),scale(u,a.springOD/2*Math.sin(t*Math.PI*16))));});
      line(helix,colors.actuator,1.4);
      const wheelAxis=rotate([0,side,0],solved?.rotation||[0,0,0]),wc=p.wheel_center;
      const tire=tireTreadGeometry(wc,wheelAxis,this.diameter,a.tireTreadWidth??null);
      // Use the known axis even if a very small positive width rounds the edge centers together.
      if(tire?.widthKnown)cylinder(...tire.edgeCenters,tire.radius,colors.tire,tire.axis);
      else if(tire)line(ring(wc,tire.axis,tire.radius),colors.tire,1.2,.75);
      // The existing concentric rim cue is illustrative, without an inferred rim width.
      line(ring(wc,wheelAxis,this.diameter*.28),colors.rim,1.2,.75);
      line([add(wc,scale(wheelAxis,-85)),add(wc,scale(wheelAxis,85))],colors.upright,1);
      for(const [key,value] of Object.entries(p))point(value,hardpointLabel(key));
    }
    const chassis=Object.entries(a.hardpoints).filter(([k])=>!isMoving(k)&&!k.startsWith('rocker_')&&k!=='damper_top');
    for(const [,p] of chassis)line([p,mirrorPoint(p)],colors.chassis,.8,.35);
    // ARB inputs describe static packaging only, independently of the solved upright.
    for(const {kind,start,end} of arbPreview.segments)line([start,end],colors.arb,kind==='bar'?3:1.8);
    for(const {side,label,position} of arbPreview.points)point(position,`${side} ARB ${label}`,colors.arb,4);
    for(const {key,label,mesh} of obstacleMeshes) {
      const fill=`${colors[key]}`;
      for(const face of mesh.faces) {
        const points=face.map(index=>mesh.vertices[index]);
        objects.push({depth:points.reduce((sum,p)=>sum+project(p)[2],0)/points.length,render:()=>{
          const screen=points.map(project);c.beginPath();screen.forEach(([x,y],index)=>index?c.lineTo(x,y):c.moveTo(x,y));c.closePath();c.globalAlpha=.18;c.fillStyle=fill;c.fill();c.globalAlpha=.5;c.strokeStyle=fill;c.lineWidth=.8;c.stroke();c.globalAlpha=1;
        }});
      }
      for(const [from,to] of mesh.edges)line([mesh.vertices[from],mesh.vertices[to]],fill,1.45,.9);
      const center=mesh.vertices.reduce((sum,p)=>add(sum,scale(p,1/mesh.vertices.length)),[0,0,0]);
      if(this.labels)objects.push({depth:project(center)[2]+2,render:()=>{const [x,y]=project(center);c.fillStyle=colors.label;c.font='10px system-ui';c.fillText(label,x+5,y-5);}});
    }
    objects.sort((a,b)=>a.depth-b.depth).forEach(o=>o.render());
    const axisOrigin=[w-44,h-53];
    for(const [p,label,color] of [[[60,0,0],'X',colors.axisX],[[0,60,0],'Y',colors.axisY],[[0,0,60],'Z',colors.axisZ]]) {
      const o=project([0,0,0]),q=project(p),dx=(q[0]-o[0])*.7,dy=(q[1]-o[1])*.7;
      c.beginPath();c.moveTo(...axisOrigin);c.lineTo(axisOrigin[0]+dx,axisOrigin[1]+dy);c.strokeStyle=color;c.lineWidth=1;c.stroke();c.fillStyle=color;c.font='8px system-ui';c.fillText(label,axisOrigin[0]+dx+3,axisOrigin[1]+dy-2);
    }
  }
}
