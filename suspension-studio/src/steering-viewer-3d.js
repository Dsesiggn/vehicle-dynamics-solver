// Three.js is used only for rendering. Every solved coordinate and spindle is
// supplied by the Python response. Screen-sized rings/arcs are symbolic glyphs.
import * as THREE from '../vendor/three/three.module.js';
import {buildSteeringScene, saeToThree, sceneBounds} from './steering-scene.js';

const clamp=(n,lo,hi)=>Math.max(lo,Math.min(hi,n));
const vector=p=>new THREE.Vector3(...saeToThree(p));
const directions={iso:[1,.85,1.15],top:[0,1,0],front:[0,0,-1],side:[1,0,0]};
function palette(){
  const css=getComputedStyle(document.body),read=(name,fallback)=>css.getPropertyValue(name).trim()||fallback;
  return {left:read('--steering-left','#a63349'),right:read('--steering-right','#178b88'),ideal:read('--steering-ideal','#bb8a34'),grid:read('--steering-grid','#dce4e7'),text:read('--steering-text','#243f49'),static:read('--steering-static','#9eacb2'),muted:read('--muted','#6a7d85'),bg:read('--steering-bg','#f6fafb')};
}

export class SteeringViewer3D {
  constructor(canvas){
    this.canvas=canvas;this.frontOnly=true;this.showIdeal=true;this.showArmLines=true;this.showArmAngles=true;this.fitIntersection=false;
    this.ready=false;this.visible=true;this.available=true;this.viewName='iso';this.onViewChange=null;this.onUnavailable=null;
    // WebGL creation deliberately throws when unavailable; caller offers Plan.
    this.renderer=new THREE.WebGLRenderer({canvas,antialias:true,alpha:false});
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio||1,2));
    this.scene=new THREE.Scene();this.group=new THREE.Group();this.scene.add(this.group);
    this.camera=new THREE.OrthographicCamera(-1,1,1,-1,.01,100000);
    this.target=new THREE.Vector3();this.direction=new THREE.Vector3(...directions.iso).normalize();
    this.camera.up.set(0,1,0);this.worldHeight=1000;this.fitHeight=1000;this.distance=10000;
    this.width=0;this.height=0;this.rebuild=true;this.fitNeeded=true;this.glyphs=[];this.labels=[];this.listeners=[];this.pointers=new Map();
    this.overlay=document.createElement('div');this.overlay.className='steering-3d-labels';
    Object.assign(this.overlay.style,{position:'absolute',inset:'0',pointerEvents:'none',overflow:'hidden'});
    canvas.parentElement.append(this.overlay);
    this.status=document.createElement('div');this.status.className='steering-3d-pose';
    Object.assign(this.status.style,{position:'absolute',left:'12px',bottom:'10px',font:'10px/1.4 system-ui,sans-serif',letterSpacing:'.04em',padding:'3px 6px',borderRadius:'4px'});
    this.overlay.append(this.status);
    canvas.tabIndex=0;canvas.style.touchAction='none';
    this.bind(canvas,'pointerdown',event=>this.pointerDown(event));
    this.bind(canvas,'pointermove',event=>this.pointerMove(event));
    for(const type of ['pointerup','pointercancel','lostpointercapture'])this.bind(canvas,type,event=>{this.pointers.delete(event.pointerId);canvas.style.cursor='grab';});
    this.bind(canvas,'contextmenu',event=>event.preventDefault());
    this.bind(canvas,'wheel',event=>{event.preventDefault();this.zoom(Math.exp(clamp(event.deltaY,-250,250)*.002));},{passive:false});
    this.bind(canvas,'keydown',event=>this.keydown(event));
    this.bind(canvas,'webglcontextlost',event=>{
      event.preventDefault();this.available=false;this.overlay.hidden=true;
      this.onUnavailable?.('The 3D graphics context was lost. Plan view is available; reload to restore 3D.');
    });
    this.bind(document,'studio-theme-change',()=>{this.rebuild=true;this.draw();});
    this.resizeObserver=new ResizeObserver(()=>{this.fitNeeded=true;this.draw();});this.resizeObserver.observe(canvas);
    canvas.style.cursor='grab';this.draw();
  }
  bind(target,type,fn,options){target.addEventListener(type,fn,options);this.listeners.push(()=>target.removeEventListener(type,fn,options));}
  setVisible(value){this.visible=Boolean(value);this.overlay.hidden=!this.visible||!this.available;if(this.visible)this.draw();}
  update(design,result,ready){
    const designKey=JSON.stringify(design);
    if(designKey!==this.designKey){this.fitNeeded=true;this.designKey=designKey;}
    if(this.fitIntersection)this.fitNeeded=true;
    this.design=design;this.result=result;this.ready=ready;this.rebuild=true;this.draw();
  }
  setView(name){
    if(!Object.hasOwn(directions,name))return;
    this.viewName=name;this.direction.set(...directions[name]).normalize();
    this.camera.up.set(...(name==='top'?[0,0,-1]:[0,1,0]));
    this.fitNeeded=true;this.onViewChange?.(name);this.draw();
  }
  fit(){this.fitNeeded=true;this.draw();}
  clearObjects(){
    this.group.traverse(object=>{object.geometry?.dispose();if(Array.isArray(object.material))object.material.forEach(m=>m.dispose());else object.material?.dispose();});
    this.group.clear();this.glyphs=[];this.labels.forEach(label=>label.element.remove());this.labels=[];
  }
  ink(style,side){
    if(['projection','ideal','rack'].includes(style))return this.colors.ideal;
    if(['static','drop','reference'].includes(style))return this.colors.static;
    return this.colors[side]||this.colors.text;
  }
  makeLine(points,style,side,closed=false){
    const converted=points.map(vector);if(closed&&converted.length)converted.push(converted[0].clone());
    const geometry=new THREE.BufferGeometry().setFromPoints(converted),dashed=['static','projection','drop','radius','ideal','reference'].includes(style);
    const common={color:this.ink(style,side),transparent:true,opacity:style==='drop'?.35:style==='static'?.48:style==='reference'?.55:style==='radius'?.72:1,depthTest:style!=='projection',depthWrite:false};
    const material=dashed?new THREE.LineDashedMaterial({...common,dashSize:5,gapSize:4}):new THREE.LineBasicMaterial(common);
    const line=new THREE.Line(geometry,material);if(dashed)line.computeLineDistances();
    line.userData.dashed=dashed;line.renderOrder=style==='projection'?2:style==='static'?0:1;this.group.add(line);return line;
  }
  makeLabel(text,position,style,side){
    const element=document.createElement('div');element.className=`steering-3d-label ${style==='projection'?'steering-3d-label-projection':''}`;element.textContent=text;
    Object.assign(element.style,{position:'absolute',font:'10px/1.3 system-ui,sans-serif',whiteSpace:'nowrap',padding:'2px 4px',borderRadius:'3px',color:this.ink(style,side),background:this.colors.bg,transform:'translate(-50%,-100%)'});
    this.overlay.append(element);const label={element,position:vector(position)};this.labels.push(label);return label;
  }
  rebuildObjects(){
    this.clearObjects();this.colors=palette();this.renderer.setClearColor(this.colors.bg);this.status.style.color=this.colors.muted;this.status.style.background=this.colors.bg;
    this.model=buildSteeringScene(this.design,this.result,{ready:this.ready,frontOnly:this.frontOnly,showIdeal:this.showIdeal,showArmLines:this.showArmLines,showArmAngles:this.showArmAngles,fitIntersection:this.fitIntersection});
    this.status.textContent=this.model.pose==='solved'?'SAE mm · rings show wheel-plane orientation, not tire size':this.model.pose==='static'?'STATIC REFERENCE · selected solved position unavailable':'';
    this.status.hidden=this.model.pose==='empty';
    if(this.model.pose==='empty')return;
    const bounds=sceneBounds(this.model.boundsPoints);
    if(bounds){
      const span=Math.max(1,...bounds.max.map((x,i)=>x-bounds.min[i]));
      const grid=new THREE.GridHelper(span*1.35,10,this.colors.grid,this.colors.grid);
      grid.position.copy(vector([(bounds.min[0]+bounds.max[0])/2,(bounds.min[1]+bounds.max[1])/2,0]));
      grid.material.transparent=true;grid.material.opacity=.45;grid.material.depthWrite=false;this.group.add(grid);
    }
    // Static comparison renders first, then the actual centerline geometry.
    for(const item of [...this.model.segments].sort((a,b)=>(a.style==='static'?0:1)-(b.style==='static'?0:1)))this.makeLine(item.points,item.style,item.side);
    for(const item of this.model.markers){
      const geometry=new THREE.BufferGeometry().setFromPoints([vector(item.position)]),material=new THREE.PointsMaterial({color:this.ink(item.style,item.side),size:item.style==='projection'?8:5,sizeAttenuation:false,depthTest:item.style!=='projection'});
      this.group.add(new THREE.Points(geometry,material));
    }
    for(const item of this.model.rings){
      const object=this.makeLine(item.unitOffsets,item.style,item.side,true);object.position.copy(vector(item.center));
      this.glyphs.push({object,radiusPx:item.radiusPx,maxRadiusMm:Infinity});
    }
    for(const item of this.model.arcs){
      const object=this.makeLine(item.unitOffsets,item.style,item.side);object.position.copy(vector(item.center));
      const label=this.makeLabel(item.label,item.center,item.style,item.side),mid=vector(item.unitOffsets[Math.floor(item.unitOffsets.length/2)]);
      this.glyphs.push({object,radiusPx:24,maxRadiusMm:item.maxRadiusMm,label,mid});
    }
    for(const item of this.model.headings){
      const object=this.makeLine([item.direction.map(v=>-v),item.direction],item.style,item.side);object.position.copy(vector(item.center));
      this.glyphs.push({object,radiusPx:item.lengthPx,maxRadiusMm:Infinity});
    }
    for(const item of this.model.labels)this.makeLabel(item.text,item.position,item.style,item.side);
    // Coordinate arrows are screen-sized reference glyphs rooted at the datum.
    for(const [name,direction] of [['+X forward',[1,0,0]],['+Y right',[0,1,0]],['+Z down',[0,0,1]]]){
      const object=this.makeLine([[0,0,0],direction],'reference'),label=this.makeLabel(name,[0,0,0],'reference');
      this.glyphs.push({object,radiusPx:56,maxRadiusMm:Infinity,label,mid:vector(direction),axis:true});
    }
  }
  fitCamera(){
    if(!this.model?.boundsPoints.length)return;
    const points=this.model.boundsPoints.map(vector),bounds=new THREE.Box3().setFromPoints(points);bounds.getCenter(this.target);
    const right=new THREE.Vector3().crossVectors(this.camera.up,this.direction).normalize(),up=new THREE.Vector3().crossVectors(this.direction,right).normalize();
    let halfWidth=0,halfHeight=0,radius=1;
    for(const point of points){const delta=point.clone().sub(this.target);halfWidth=Math.max(halfWidth,Math.abs(delta.dot(right)));halfHeight=Math.max(halfHeight,Math.abs(delta.dot(up)));radius=Math.max(radius,delta.length());}
    // Reserve fixed screen space for the symbolic wheel rings, arc labels and
    // datum axes. Their visual radii are deliberately not physical dimensions.
    const availableWidth=Math.max(40,this.width-130),availableHeight=Math.max(40,this.height-130);
    this.worldHeight=Math.max(2*halfHeight*this.height/availableHeight,2*halfWidth*this.height/availableWidth,1);
    this.fitHeight=this.worldHeight;this.distance=radius*4+this.worldHeight;this.fitNeeded=false;
  }
  applyCamera(){
    const aspect=this.width/this.height;
    this.camera.left=-this.worldHeight*aspect/2;this.camera.right=this.worldHeight*aspect/2;this.camera.top=this.worldHeight/2;this.camera.bottom=-this.worldHeight/2;
    this.camera.near=Math.max(.0001,this.distance*1e-6);this.camera.far=Math.max(this.distance*5,this.worldHeight*20);
    this.camera.position.copy(this.target).addScaledVector(this.direction,this.distance);this.camera.lookAt(this.target);this.camera.updateProjectionMatrix();this.camera.updateMatrixWorld();
  }
  draw(){
    if(!this.available||!this.visible)return;
    this.overlay.hidden=Boolean(this.canvas.hidden);if(this.canvas.hidden)return;
    const rect=this.canvas.getBoundingClientRect();if(rect.width<1||rect.height<1)return;
    if(rect.width!==this.width||rect.height!==this.height){this.width=rect.width;this.height=rect.height;this.renderer.setSize(this.width,this.height,false);}
    const options=[this.frontOnly,this.showIdeal,this.showArmLines,this.showArmAngles,this.fitIntersection].join('|');
    if(options!==this.options){if(this.options?.split('|')[0]!==String(this.frontOnly)||this.options?.split('|')[4]!==String(this.fitIntersection))this.fitNeeded=true;this.options=options;this.rebuild=true;}
    if(this.rebuild){this.rebuildObjects();this.rebuild=false;}
    if(this.fitNeeded)this.fitCamera();this.applyCamera();
    const mmPerPixel=this.worldHeight/this.height;
    for(const glyph of this.glyphs){
      const radius=Math.min(glyph.radiusPx*mmPerPixel,glyph.maxRadiusMm);glyph.object.scale.setScalar(radius);
      if(glyph.label){
        glyph.label.position.copy(glyph.object.position).addScaledVector(glyph.mid,radius*(glyph.axis?1:1.35));
        if(glyph.axis){const a=glyph.object.position.clone().project(this.camera),b=glyph.label.position.clone().project(this.camera);glyph.label.hide=Math.hypot((b.x-a.x)*this.width/2,(b.y-a.y)*this.height/2)<14;}
      }
    }
    this.group.traverse(object=>{if(object.userData.dashed){const scale=object.scale.x||1;object.material.dashSize=5*mmPerPixel/scale;object.material.gapSize=4*mmPerPixel/scale;}});
    this.renderer.render(this.scene,this.camera);
    const occupied=[];
    for(const label of this.labels){
      const p=label.position.clone().project(this.camera),x=(p.x+1)*this.width/2,y=(1-p.y)*this.height/2;
      label.element.hidden=Boolean(label.hide)||p.z< -1||p.z>1||x<0||x>this.width||y<0||y>this.height;
      if(!label.element.hidden){
        const half=Math.min(label.element.offsetWidth/2+4,this.width/2),height=label.element.offsetHeight+3;
        // A small screen-space label adjustment only; geometry stays fixed.
        // Angle labels are placed first and wheel/datum captions move around them.
        let placement=null,bestOverlap=Infinity;
        for(const [dx,dy] of [[0,-7],[0,-27],[0,18],[-42,-7],[42,-7],[0,-47]]){
          const px=clamp(x+dx,half,this.width-half),py=clamp(y+dy,height+3,this.height-31),box={left:px-half,right:px+half,top:py-height,bottom:py+2};
          const overlap=occupied.reduce((sum,b)=>sum+Math.max(0,Math.min(box.right,b.right)-Math.max(box.left,b.left))*Math.max(0,Math.min(box.bottom,b.bottom)-Math.max(box.top,b.top)),0);
          if(overlap<bestOverlap){placement={px,py,box};bestOverlap=overlap;}if(overlap===0)break;
        }
        occupied.push(placement.box);label.element.style.left=`${placement.px}px`;label.element.style.top=`${placement.py}px`;
      }
    }
  }
  orbit(dx,dy){
    let azimuth=Math.atan2(this.direction.x,this.direction.z),polar=Math.acos(clamp(this.direction.y,-1,1));
    azimuth-=dx*.008;polar=clamp(polar-dy*.008,.005,Math.PI-.005);
    this.direction.set(Math.sin(polar)*Math.sin(azimuth),Math.cos(polar),Math.sin(polar)*Math.cos(azimuth));
    this.camera.up.set(0,1,0);this.viewName='orbit';this.onViewChange?.('orbit');this.draw();
  }
  pan(dx,dy){
    const right=new THREE.Vector3().crossVectors(this.camera.up,this.direction).normalize(),up=new THREE.Vector3().crossVectors(this.direction,right).normalize(),scale=this.worldHeight/Math.max(1,this.height);
    this.target.addScaledVector(right,-dx*scale).addScaledVector(up,dy*scale);this.draw();
  }
  zoom(factor){this.worldHeight=clamp(this.worldHeight*factor,this.fitHeight*1e-4,this.fitHeight*1e4);this.draw();}
  pointerDown(event){
    event.preventDefault();this.canvas.focus({preventScroll:true});this.canvas.setPointerCapture(event.pointerId);
    this.pointers.set(event.pointerId,{x:event.clientX,y:event.clientY,pan:event.shiftKey||event.button===1||event.button===2});this.canvas.style.cursor='grabbing';
  }
  pointerMove(event){
    const previous=this.pointers.get(event.pointerId);if(!previous)return;
    const before=[...this.pointers.values()];this.pointers.set(event.pointerId,{...previous,x:event.clientX,y:event.clientY});
    const after=[...this.pointers.values()];
    if(before.length>=2){
      const distance=p=>Math.hypot(p[0].x-p[1].x,p[0].y-p[1].y),oldDistance=distance(before),newDistance=distance(after);
      if(oldDistance>1&&newDistance>1)this.zoom(oldDistance/newDistance);
      this.pan((after[0].x+after[1].x-before[0].x-before[1].x)/2,(after[0].y+after[1].y-before[0].y-before[1].y)/2);
    }else if(previous.pan||event.shiftKey)this.pan(event.clientX-previous.x,event.clientY-previous.y);
    else this.orbit(event.clientX-previous.x,event.clientY-previous.y);
  }
  keydown(event){
    const delta={ArrowLeft:[-12,0],ArrowRight:[12,0],ArrowUp:[0,-12],ArrowDown:[0,12]}[event.key];
    if(delta){event.preventDefault();event.shiftKey?this.pan(...delta):this.orbit(...delta);}
    else if(['+','=','-','_'].includes(event.key)){event.preventDefault();this.zoom(['+','='].includes(event.key)?.85:1/.85);}
    else if(event.key.toLowerCase()==='f'){event.preventDefault();this.fit();}
    else if(event.key==='Home'){event.preventDefault();this.setView('iso');}
  }
  dispose(){this.listeners.forEach(remove=>remove());this.resizeObserver.disconnect();this.clearObjects();this.renderer.dispose();this.overlay.remove();}
}
