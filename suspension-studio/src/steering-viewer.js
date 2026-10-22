// Display only: solved motion and derived constructions have independent pure modules.
import { steeringConstruction } from './steering-geometry.js';
const finitePoint = p => Array.isArray(p) && p.length === 3 && p.every(Number.isFinite);
const finite = Number.isFinite;
const degrees = Math.PI / 180;
const signed = value => `${value > 0 ? '+' : ''}${value.toFixed(1)}°`;

function canvasContext(canvas) {
  const bounds = canvas.getBoundingClientRect(), ratio = window.devicePixelRatio || 1;
  const width = Math.max(1, bounds.width), height = Math.max(1, bounds.height);
  canvas.width = Math.round(width * ratio); canvas.height = Math.round(height * ratio);
  const context = canvas.getContext('2d');
  context.setTransform(ratio, 0, 0, ratio, 0, 0); context.clearRect(0, 0, width, height);
  return { context, width, height };
}
function colors() {
  const style = getComputedStyle(document.body), color = name => style.getPropertyValue(name).trim();
  return { left: color('--steering-left'), right: color('--steering-right'), ideal: color('--steering-ideal'), grid: color('--steering-grid'), text: color('--steering-text'), static: color('--steering-static'), muted: color('--muted'), bg: color('--steering-bg') };
}
function stroke(c, points, color, width = 1.5, dash = []) {
  if (points.length < 2) return;
  c.beginPath(); points.forEach(([x,y],i)=> i ? c.lineTo(x,y) : c.moveTo(x,y));
  c.strokeStyle = color; c.lineWidth = width; c.setLineDash(dash); c.stroke(); c.setLineDash([]);
}
function point(c, [x,y], color, radius = 3) {
  c.beginPath(); c.arc(x,y,radius,0,Math.PI*2); c.fillStyle = color; c.fill();
}
function label(c, text, x, y, color, align = 'center', size = 10) {
  c.font = `${size}px system-ui,sans-serif`; c.fillStyle = color; c.textAlign = align; c.fillText(text,x,y);
}
function wheel(c, p, heading, color, dashed = false) {
  c.save();c.translate(p[0],p[1]);c.rotate(heading*degrees);
  c.strokeStyle=color;c.lineWidth=dashed?1.7:2;c.setLineDash(dashed?[4,3]:[]);
  if(dashed){c.beginPath();c.moveTo(0,-29);c.lineTo(0,29);c.stroke();}
  else {c.fillStyle=color;c.globalAlpha=.09;c.fillRect(-7,-23,14,46);c.globalAlpha=1;c.strokeRect(-7,-23,14,46);c.beginPath();c.moveTo(0,12);c.lineTo(0,-15);c.lineTo(-3,-10);c.moveTo(0,-15);c.lineTo(3,-10);c.stroke();}
  c.restore();
}

// Clip an infinite screen-space line to the drawable viewport, without limiting
// the underlying physical intersection or changing the construction geometry.
function extendedLine(c, a, b, color, width, height) {
  const d=[b[0]-a[0],b[1]-a[1]],length=Math.hypot(...d);
  if(!Number.isFinite(length)||length===0)return;
  const v=d.map(x=>x/length),lo=[14,44],hi=[width-14,height-54];
  let enter=-Infinity,leave=Infinity;
  for(let j=0;j<2;j++) {
    if(Math.abs(v[j])<1e-14){if(a[j]<lo[j]||a[j]>hi[j])return;continue;}
    const t1=(lo[j]-a[j])/v[j],t2=(hi[j]-a[j])/v[j];
    enter=Math.max(enter,Math.min(t1,t2));leave=Math.min(leave,Math.max(t1,t2));
  }
  if(enter<=leave)stroke(c,[[a[0]+enter*v[0],a[1]+enter*v[1]],[a[0]+leave*v[0],a[1]+leave*v[1]]],color,1.1,[7,5]);
}

function angleArc(c, outer, axis, inner, value, side, color, background, width, height) {
  if(!finite(value))return;
  const a=[axis[0]-outer[0],axis[1]-outer[1]],b=[inner[0]-outer[0],inner[1]-outer[1]];
  const start=Math.atan2(a[1],a[0]),sweep=Math.atan2(a[0]*b[1]-a[1]*b[0],a[0]*b[0]+a[1]*b[1]);
  const radius=Math.min(25,Math.max(12,Math.min(Math.hypot(...a),Math.hypot(...b))*.65));
  c.beginPath();c.moveTo(...outer);c.arc(...outer,radius,start,start+sweep,sweep<0);c.closePath();
  c.fillStyle=color;c.globalAlpha=.11;c.fill();c.globalAlpha=1;
  c.beginPath();c.arc(...outer,radius,start,start+sweep,sweep<0);c.strokeStyle=color;c.lineWidth=1.5;c.stroke();
  const caption=`α${side==='left'?'L':'R'} ${value.toFixed(1)}°`;
  c.font='10px system-ui,sans-serif';
  const halfWidth=c.measureText(caption).width/2+4,mid=start+sweep/2;
  const x=Math.max(halfWidth+5,Math.min(width-halfWidth-5,outer[0]+(radius+30)*Math.cos(mid)));
  const y=Math.max(54,Math.min(height-59,outer[1]+(radius+30)*Math.sin(mid)));
  c.fillStyle=background;c.fillRect(x-halfWidth,y-10,2*halfWidth,15);
  label(c,caption,x,y+1,color,'center',10);
}

export class SteeringViewer {
  constructor(canvas) {
    this.canvas=canvas;this.frontOnly=false;this.showIdeal=true;this.showArmLines=true;this.showArmAngles=true;this.fitIntersection=false;this.ready=false;
    new ResizeObserver(()=>this.draw()).observe(canvas);
    document.addEventListener('studio-theme-change',()=>this.draw());
  }
  update(design, result, ready) {this.design=design;this.result=result;this.ready=ready;this.draw();}
  draw() {
    const {context:c,width:w,height:h}=canvasContext(this.canvas), color=colors();
    if(!this.ready||!this.design)return;
    const d=this.design,result=this.result?.ok?this.result:null;
    const construction=this.result?.construction??steeringConstruction(d,result),neutral=steeringConstruction(d),intersection=construction.intersection;
    const centers={left:[0,-d.frontTrack/2,d.left.wheelCenterZ],right:[0,d.frontTrack/2,d.right.wheelCenterZ]};
    const rear={left:[-d.wheelbase,-d.rearTrack/2,0],right:[-d.wheelbase,d.rearTrack/2,0]};
    const boundsPoints=Object.keys(centers).flatMap(side=>[centers[side],d[side].axisUpper,d[side].axisLower,d[side].tieInner,d[side].tieOuter,...(result?[result[side].outer,result[side].inner,result[side].wheelCenter]:[])]).filter(finitePoint);
    if(!this.frontOnly)boundsPoints.push(rear.left,rear.right);
    if(this.fitIntersection&&this.showArmLines&&intersection.status==='finite')boundsPoints.push(intersection.point);
    const xs=boundsPoints.map(p=>p[0]),ys=boundsPoints.map(p=>p[1]);
    const minX=Math.min(...xs),maxX=Math.max(...xs),minY=Math.min(...ys),maxY=Math.max(...ys);
    const spanX=Math.max(maxX-minX,this.frontOnly?d.frontTrack*.36:1),spanY=Math.max(1,maxY-minY);
    const scale=Math.min((w-115)/spanY,(h-115)/spanX),cx=(minX+maxX)/2,cy=(minY+maxY)/2;
    const project=p=>[w/2+(p[1]-cy)*scale,h/2-(p[0]-cx)*scale];
    const step=10**Math.floor(Math.log10(Math.max(spanX,spanY)/5));
    for(let x=Math.floor(minX/step)*step;x<=maxX+step;x+=step)stroke(c,[project([x,minY-step,0]),project([x,maxY+step,0])],color.grid,.6);
    for(let y=Math.floor(minY/step)*step;y<=maxY+step;y+=step)stroke(c,[project([minX-step,y,0]),project([maxX+step,y,0])],color.grid,.6);
    stroke(c,[project([maxX+spanX*.06,0,0]),project([minX-spanX*.06,0,0])],color.static,1,[4,5]);
    stroke(c,[project(centers.left),project(centers.right)],color.static,1,[5,4]);
    if(!this.frontOnly){stroke(c,[project(rear.left),project(rear.right)],color.static,1,[5,4]);for(const side of ['left','right'])wheel(c,project(rear[side]),0,color.static);const midpoint=project([-d.wheelbase,0,0]);label(c,`REAR · ${d.rearTrack.toLocaleString()} mm`,midpoint[0],midpoint[1]-18,color.muted,'center',9);}
    if(this.showArmLines) {
      for(const side of ['left','right']) {
        const arm=construction[side];
        if(finitePoint(arm.planAxisPoint)&&finitePoint(arm.outer))extendedLine(c,project(arm.planAxisPoint),project(arm.outer),color[side],w,h);
      }
      if(intersection.status==='finite') {
        const p=project(intersection.point),visible=p[0]>=14&&p[0]<=w-14&&p[1]>=44&&p[1]<=h-54;
        if(visible){
          stroke(c,[[p[0]-5,p[1]-5],[p[0]+5,p[1]+5]],color.text,1.7);
          stroke(c,[[p[0]-5,p[1]+5],[p[0]+5,p[1]-5]],color.text,1.7);
          label(c,'Arm-line intersection',Math.max(75,Math.min(w-75,p[0])),p[1]+18,color.text,'center',10);
        }else label(c,'INTERSECTION OUTSIDE VIEW · FIT INTERSECTION TO SEE IT',w/2,h-48,color.muted,'center',8);
      }
    }
    const innerL=result?result.left.inner:d.left.tieInner,innerR=result?result.right.inner:d.right.tieInner;
    stroke(c,[project(d.left.tieInner),project(d.right.tieInner)],color.static,5);
    stroke(c,[project(innerL),project(innerR)],color.ideal,4);
    for(const side of ['left','right']) {
      const a=d[side],s=side==='left'?-1:1,actual=result?.[side],ink=color[side];
      const outer=actual?.outer||a.tieOuter,inner=actual?.inner||a.tieInner,center=actual?.wheelCenter||centers[side];
      const heading=actual?.headingDeg||0,staticHeading=-s*a.staticToeDeg;
      if(finitePoint(neutral[side].planAxisPoint))stroke(c,[project(neutral[side].planAxisPoint),project(a.tieOuter)],color.static,1.5);
      stroke(c,[project(a.tieInner),project(a.tieOuter)],color.static,1.2,[3,3]);
      wheel(c,project(centers[side]),staticHeading,color.static,true);
      stroke(c,[project(a.axisLower),project(a.axisUpper)],ink,3);
      if(finitePoint(construction[side].planAxisPoint))stroke(c,[project(construction[side].planAxisPoint),project(outer)],ink,3);
      stroke(c,[project(inner),project(outer)],ink,2.3);
      for(const p of [a.axisLower,a.axisUpper,outer,inner])point(c,project(p),ink,3.4);
      if(finitePoint(construction[side].planAxisPoint))point(c,project(construction[side].planAxisPoint),ink,3.4);
      point(c,project(center),ink,2.5);
      wheel(c,project(center),actual?heading:staticHeading,ink);
      const screen=project(center);
      label(c,`${side==='left'?'L':'R'} ${actual?signed(heading):'static'}`,screen[0],screen[1]-36,ink,'center',11);
      if(this.showIdeal&&result?.metrics?.insideSide&&finite(result.metrics.idealOutsideDeg)) {
        const m=result.metrics,sign=m.turn==='right'?1:-1;
        const ideal=side===m.insideSide?sign*m.insideDeg:sign*m.idealOutsideDeg;
        wheel(c,project(centers[side]),ideal,color.ideal,true);
      }
    }
    if(this.showArmAngles)for(const side of ['left','right']) {
      const arm=construction[side];
      if(finitePoint(arm.planAxisPoint)&&finitePoint(arm.outer)&&finitePoint(arm.inner))angleArc(c,project(arm.outer),project(arm.planAxisPoint),project(arm.inner),arm.planAngleDeg,side,color[side],color.bg,w,h);
    }
    label(c,`FRONT · nominal track ${d.frontTrack.toLocaleString()} mm`,w/2,26,color.muted,'center',10);
    label(c,this.frontOnly?'FRONT LINKAGE · PLAN PROJECTION':`WHEELBASE ${d.wheelbase.toLocaleString()} mm`,w/2,h-14,color.muted,'center',9);
    stroke(c,[[22,58],[22,29],[18,35],[22,29],[26,35]],color.muted,1.2);label(c,'+X',22,20,color.muted,'center',9);
    if(!result)label(c,'STATIC GEOMETRY · SELECTED POSITION UNAVAILABLE',w/2,h-31,color.ideal,'center',9);
  }
}

export class SteeringChart {
  constructor(canvas){this.canvas=canvas;this.rows=[];this.angle=0;this.mode='headings';new ResizeObserver(()=>this.draw()).observe(canvas);document.addEventListener('studio-theme-change',()=>this.draw());}
  update(rows,angle){this.rows=rows||[];this.angle=angle;this.draw();}
  draw(){
    const {context:c,width:w,height:h}=canvasContext(this.canvas),color=colors();
    const series=this.mode==='toe'?[{value:r=>r.metrics?.deltaTotalToeInDeg,ink:color.left}]:this.mode==='error'?[{value:r=>r.metrics?.ackermannErrorDeg,ink:color.ideal}]:[{value:r=>r.left?.headingDeg,ink:color.left},{value:r=>r.right?.headingDeg,ink:color.right}];
    const good=this.rows.filter(r=>r.ok&&series.some(line=>finite(line.value(r))));
    if(!good.length){label(c,'The complete sweep will appear here.',w/2,h/2,color.muted,'center',12);return;}
    const left=45,right=w-16,top=17,bottom=h-33;
    const limit=Math.max(5,...this.rows.map(r=>Math.abs(r.steeringWheelDeg))),extent=Math.max(...good.flatMap(r=>series.map(line=>line.value(r)).filter(finite).map(Math.abs)));
    const tickBase=10**Math.floor(Math.log10(Math.max(.01,extent))),yMax=extent>0?Math.max(.01,Math.ceil(extent/tickBase)*tickBase):1;
    const project=(x,y)=>[left+(x+limit)/(2*limit)*(right-left),bottom-(y+yMax)/(2*yMax)*(bottom-top)];
    for(let i=0;i<=4;i++) {
      const y=-yMax+i*yMax/2,py=project(0,y)[1];stroke(c,[[left,py],[right,py]],color.grid,.8);label(c,`${+y.toFixed(yMax<1?3:2)}°`,left-8,py+3,color.muted,'right',9);
      const x=-limit+i*limit/2,px=project(x,0)[0];stroke(c,[[px,top],[px,bottom]],color.grid,.8);label(c,`${+x.toFixed(1)}°`,px,bottom+18,color.muted,'center',9);
    }
    stroke(c,[project(-limit,0),project(limit,0)],color.static,1);
    for(const line of series) {
      let segment=[];
      for(const row of this.rows){if(row.ok&&finite(line.value(row))){segment.push(project(row.steeringWheelDeg,line.value(row)));}else{stroke(c,segment,line.ink,2);segment=[];}}
      stroke(c,segment,line.ink,2);
    }
    const x=project(this.angle,0)[0];stroke(c,[[x,top],[x,bottom]],color.muted,1,[3,4]);
    const selected=good.find(r=>Math.abs(r.steeringWheelDeg-this.angle)<1e-8);
    if(selected)for(const line of series)if(finite(line.value(selected)))point(c,project(this.angle,line.value(selected)),line.ink,3.5);
  }
}
