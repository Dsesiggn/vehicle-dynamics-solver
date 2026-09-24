import {linkPairs,isMoving} from './model.js';
import {add,sub,scale,unit,cross,rotate} from './solver.js';
const COLORS={arm:'#71889d',actuator:'#98627f',steering:'#ad976c',upright:'#8f86a1',point:'#a98dbc',chassis:'#b6b8c7'};
export class SuspensionViewer {
  constructor(canvas) {
    this.canvas=canvas;this.ctx=canvas.getContext('2d');this.yaw=-.48;this.pitch=.32;this.zoom=1;this.labels=false;
    new ResizeObserver(()=>this.draw()).observe(canvas);
    canvas.addEventListener('pointerdown',e=>{this.drag=[e.clientX,e.clientY];canvas.setPointerCapture(e.pointerId);});
    canvas.addEventListener('pointermove',e=>{if(!this.drag)return;this.yaw+=(e.clientX-this.drag[0])*.007;this.pitch=Math.max(-1.5,Math.min(1.5,this.pitch+(e.clientY-this.drag[1])*.007));this.drag=[e.clientX,e.clientY];this.draw();});
    canvas.addEventListener('pointerup',()=>this.drag=null);canvas.addEventListener('pointercancel',()=>this.drag=null);
    canvas.addEventListener('wheel',e=>{e.preventDefault();this.zoom=Math.max(.5,Math.min(2.5,this.zoom*Math.exp(-e.deltaY*.001)));this.draw();},{passive:false});
    canvas.addEventListener('keydown',e=>{if(!['ArrowLeft','ArrowRight','ArrowUp','ArrowDown','+','-','='].includes(e.key))return;e.preventDefault();if(e.key==='ArrowLeft')this.yaw-=.12;if(e.key==='ArrowRight')this.yaw+=.12;if(e.key==='ArrowUp')this.pitch=Math.min(1.5,this.pitch+.1);if(e.key==='ArrowDown')this.pitch=Math.max(-1.5,this.pitch-.1);if(e.key==='+'||e.key==='=')this.zoom=Math.min(2.5,this.zoom*1.1);if(e.key==='-')this.zoom=Math.max(.5,this.zoom/1.1);this.draw();});
  }
  view(name) {this.yaw=name==='iso'?-.48:0;this.pitch=name==='top'?Math.PI/2:name==='front'?0:.32;this.zoom=1;this.draw();}
  update(a,diameter,motion=null) {this.axle=a;this.diameter=diameter;this.motion=motion;this.draw();}
  draw() {
    if(!this.axle)return;
    const c=this.ctx,canvas=this.canvas,rect=canvas.getBoundingClientRect(),w=rect.width,h=rect.height,dpr=window.devicePixelRatio||1;
    canvas.width=Math.round(w*dpr);canvas.height=Math.round(h*dpr);c.setTransform(dpr,0,0,dpr,0,0);c.clearRect(0,0,w,h);
    const a=this.axle,s=Math.min(w/(a.track+this.diameter*.5+180),h/(this.diameter+350))*this.zoom;
    const project=p=>{const x=p[0]*Math.cos(this.yaw)-p[1]*Math.sin(this.yaw),depth=p[0]*Math.sin(this.yaw)+p[1]*Math.cos(this.yaw);return [w/2+x*s,h*.64-(p[2]-this.diameter*.3)*s*Math.cos(this.pitch)+depth*s*Math.sin(this.pitch),depth*Math.cos(this.pitch)+p[2]*Math.sin(this.pitch)];};
    const drawLine=(points,color,width=1,alpha=1,dash=[])=>{c.beginPath();points.forEach((p,i)=>{const [x,y]=project(p);i?c.lineTo(x,y):c.moveTo(x,y);});c.strokeStyle=color;c.globalAlpha=alpha;c.lineWidth=width;c.setLineDash(dash);c.stroke();c.setLineDash([]);c.globalAlpha=1;};
    for(let x=-2000;x<=2000;x+=100)drawLine([[x,-900,0],[x,900,0]],'#bcc0cb',.6,.3);
    for(let y=-900;y<=900;y+=100)drawLine([[-2000,y,0],[2000,y,0]],'#bcc0cb',.6,.3);
    drawLine([[-1800,0,0],[1800,0,0]],'#b0acbc',.8,.5);
    const objects=[];
    const line=(points,color,width=2,alpha=1)=>objects.push({depth:points.reduce((sum,p)=>sum+project(p)[2],0)/points.length,render:()=>drawLine(points,color,width,alpha)});
    const point=(p,label)=>objects.push({depth:project(p)[2]+1,render:()=>{const [x,y]=project(p);c.beginPath();c.arc(x,y,2.6,0,Math.PI*2);c.fillStyle='#faf8fc';c.fill();c.strokeStyle=COLORS.point;c.lineWidth=1.2;c.stroke();if(this.labels){c.fillStyle='#766b83';c.font='7px system-ui';c.fillText(label,x+5,y-5);}}});
    const cylinder=(p1,p2,radius,color)=>{
      const axis=unit(sub(p2,p1)),v=unit(cross(axis,Math.abs(axis[2])>.9?[1,0,0]:[0,0,1])),u=cross(axis,v);
      const ring=(p,r)=>Array.from({length:33},(_,i)=>add(p,add(scale(v,r*Math.cos(i*Math.PI/16)),scale(u,r*Math.sin(i*Math.PI/16)))));
      line(ring(p1,radius),color,1.2,.75);line(ring(p2,radius),color,1.2,.75);
      for(let i=0;i<8;i++) {const offset=add(scale(v,radius*Math.cos(i*Math.PI/4)),scale(u,radius*Math.sin(i*Math.PI/4)));line([add(p1,offset),add(p2,offset)],color,.7,.35);}
    };
    for(const side of [-1,1]) {
      const solved=this.motion?.ok?(side===1?this.motion.left:this.motion.right):null;
      const local=solved?.points||a.hardpoints, mirror=p=>[p[0]*side,p[1],p[2]],p=Object.fromEntries(Object.entries(local).map(([k,v])=>[k,mirror(v)]));
      for(const [i,o] of linkPairs(a))line([p[i],p[o]],i==='tie_rod_inner'?COLORS.steering:COLORS.arm,2.7);
      const upright=Object.entries(p).filter(([k])=>isMoving(k)&&k!=='wheel_center'&&k!=='actuation_outer').map(([,v])=>v);
      for(const v of upright)line([v,p.wheel_center],COLORS.upright,1.4,.6);
      if(a.topology==='double-wishbone') {line([p.uca_front,p.uca_rear],COLORS.arm,1,.4);line([p.lca_front,p.lca_rear],COLORS.arm,1,.4);}
      let damperStart,damperEnd;
      if(a.topology==='macpherson'){damperStart=p.strut_top;damperEnd=p.strut_bottom;}
      else if(a.actuation==='direct'){damperStart=p.damper_top;damperEnd=p.actuation_outer;}
      else {
        line([p.actuation_outer,p.rocker_rod],COLORS.actuator,2.7);
        line([p.rocker_pivot,p.rocker_rod,p.rocker_damper,p.rocker_pivot],COLORS.actuator,2);
        damperStart=p.damper_top;damperEnd=p.rocker_damper;
      }
      cylinder(damperStart,damperEnd,a.damperOD/2,COLORS.actuator);
      const axis=unit(sub(damperEnd,damperStart)),v=unit(cross(axis,[0,1,0])),u=cross(axis,v);
      const helix=Array.from({length:161},(_,i)=>{const t=i/160;return add(add(damperStart,scale(sub(damperEnd,damperStart),.12+.7*t)),add(scale(v,a.springOD/2*Math.cos(t*Math.PI*16)),scale(u,a.springOD/2*Math.sin(t*Math.PI*16))));});
      line(helix,COLORS.actuator,1.4);
      const wheelAxis=mirror(rotate([1,0,0],solved?.rotation||[0,0,0])),wc=p.wheel_center;
      cylinder(add(wc,scale(wheelAxis,-65)),add(wc,scale(wheelAxis,65)),this.diameter/2,'#a0a2b6');
      cylinder(add(wc,scale(wheelAxis,-50)),add(wc,scale(wheelAxis,50)),this.diameter*.28,'#aaa4b7');
      line([add(wc,scale(wheelAxis,-85)),add(wc,scale(wheelAxis,85))],COLORS.upright,1);
      for(const [key,value] of Object.entries(p))point(value,key.replaceAll('_',' '));
    }
    const chassis=Object.entries(a.hardpoints).filter(([k])=>!isMoving(k)&&!k.startsWith('rocker_')&&k!=='damper_top');
    for(const [,p] of chassis)line([p,[-p[0],p[1],p[2]]],COLORS.chassis,.8,.35);
    objects.sort((a,b)=>a.depth-b.depth).forEach(o=>o.render());
    const axisOrigin=[w-44,h-53];
    for(const [p,label,color] of [[[60,0,0],'X','#ae8497'],[[0,60,0],'Y','#8c9b86'],[[0,0,60],'Z','#8b93af']]) {
      const o=project([0,0,0]),q=project(p),dx=(q[0]-o[0])*.7,dy=(q[1]-o[1])*.7;
      c.beginPath();c.moveTo(...axisOrigin);c.lineTo(axisOrigin[0]+dx,axisOrigin[1]+dy);c.strokeStyle=color;c.lineWidth=1;c.stroke();c.fillStyle=color;c.font='8px system-ui';c.fillText(label,axisOrigin[0]+dx+3,axisOrigin[1]+dy-2);
    }
  }
}
