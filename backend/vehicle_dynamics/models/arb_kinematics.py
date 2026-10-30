"""Declared straight transverse U-bar; no stiffness/force inference."""
import math
import numpy as np
from scipy.spatial.transform import Rotation
from .suspension import actuator_velocity


def _fail(reason):
    return {"available":False,"reason":reason}


def _pickup(axle,pose,settings,velocity=None):
    reflection=np.array([1.,-1. if pose["side"]=="right" else 1.,1.])
    p={key:np.asarray(value)*reflection for key,value in axle["hardpoints"].items()}
    q={key:np.asarray(value) for key,value in pose["points"].items()}
    p0=np.asarray(axle["antiRollBar"]["points"]["link_pickup"],dtype=float)*reflection
    body=settings.get("arbAttachment")
    if body=="upright":
        rotation=Rotation.from_rotvec(pose["rotation"]).as_matrix()
        point=q["wheel_center"]+rotation@(p0-p["wheel_center"])
        rate=np.asarray(velocity["linear"])+np.cross(velocity["angular"],point-q["wheel_center"]) if velocity else None
        return point,rate
    if body in ("lower-wishbone","upper-wishbone"):
        if axle["topology"]!="double-wishbone":raise ValueError("Wishbone ARB pickup requires double-wishbone topology.")
        arm="lca" if body=="lower-wishbone" else "uca"
        origin=p[arm+"_front"];k=p[arm+"_rear"]-origin
        if np.linalg.norm(k)<1e-7:raise ValueError("Wishbone pivot axis is unresolved.")
        k/=np.linalg.norm(k)
        v=p[arm+"_outer"]-origin;v-=k*(k@v)
        w=q[arm+"_outer"]-origin;w-=k*(k@w)
        if min(np.linalg.norm(v),np.linalg.norm(w))<1e-7:raise ValueError("Wishbone pickup carrier is singular.")
        angle=math.atan2(float(k@np.cross(v,w)),float(v@w))
        point=origin+Rotation.from_rotvec(k*angle).apply(p0-origin)
        rate=None
        if velocity:
            ball_velocity=np.asarray(velocity["linear"])+np.cross(velocity["angular"],q[arm+"_outer"]-q["wheel_center"])
            tangent=np.cross(k,w);angular_rate=float(tangent@ball_velocity/(tangent@tangent))
            rate=np.cross(k*angular_rate,point-origin)
        return point,rate
    if body=="rocker":
        if not pose["actuator"]["ok"] or pose["actuator"].get("angleRad") is None:raise ValueError("A solved rocker and explicit rocker axis are required for this ARB pickup.")
        k=np.asarray(settings["rockerAxis"],dtype=float)*reflection;k/=np.linalg.norm(k)
        origin=p["rocker_pivot"]
        point=origin+Rotation.from_rotvec(k*pose["actuator"]["angleRad"]).apply(p0-origin)
        rate=None
        if velocity:
            derivative=actuator_velocity(axle,pose,velocity,rocker_axis=settings["rockerAxis"])
            if not derivative["ok"]:raise ValueError(derivative["reason"])
            rate=np.cross(k*derivative["rockerAngleRateRad"],point-origin)
        return point,rate
    raise ValueError("Select the rigid body carrying the ARB suspension pickup.")


def _arm(bend,tip0,pickup0,pickup,previous=0.):
    k=np.array([0.,1.,0.]);arm=tip0-bend
    parallel=k*(k@arm);v=arm-parallel;w=np.cross(k,v)
    radius=float(np.linalg.norm(v));length=float(np.linalg.norm(tip0-pickup0))
    if min(radius,length)<1e-7:raise ValueError("ARB arm or drop link is degenerate.")
    d0=bend+parallel-pickup0;neutral=float(d0@w)
    if abs(neutral)<1e-10*max(1.,np.linalg.norm(d0)*radius):raise ValueError("ARB neutral geometry is at an unresolved toggle.")
    d=bend+parallel-pickup;a=float(d@v);b=float(d@w)
    c=(length**2-float(d@d)-radius**2)/2;amplitude=math.hypot(a,b)
    if amplitude==0 or amplitude**2-c**2<=1e-12*amplitude**2:raise ValueError("ARB drop-link closure is inaccessible or at a toggle.")
    phase=math.atan2(b,a);offset=math.acos(max(-1.,min(1.,c/amplitude)))
    candidates=[angle for angle in (phase-offset,phase+offset) if (-a*math.sin(angle)+b*math.cos(angle))*neutral>0]
    if len(candidates)!=1:raise ValueError("ARB branch is unresolved.")
    angle=candidates[0]+2*math.pi*round((previous-candidates[0])/(2*math.pi))
    if abs(angle)>math.pi/2 or abs(angle-previous)>math.pi/4:raise ValueError("ARB motion leaves the supported nearby assembly branch.")
    tip=bend+Rotation.from_rotvec(k*angle).apply(arm)
    residual=abs(float(np.linalg.norm(tip-pickup))-length)
    if residual>1e-6:raise ValueError("ARB drop-link closure exceeds tolerance.")
    return {"angleRad":float(angle),"angleDeg":math.degrees(angle),"arcMm":radius*angle,"radiusMm":radius,"tip":tip.tolist(),"pickup":pickup.tolist(),"bend":bend.tolist(),"residualMm":residual}


def solve_arb(axle,pose,settings):
    arb=axle.get("antiRollBar")
    if not arb or not arb.get("enabled"):return _fail("Enable and define this axle's U-bar geometry.")
    if settings.get("arbModel")!="straight-transverse":return _fail("Select the straight transverse U-bar model; a static preview alone does not select its constraints.")
    if not pose.get("ok"):return _fail("A solved suspension corner is required.")
    try:
        points=arb.get("points",{})
        if any(not isinstance(points.get(key),list) or len(points[key])!=3 or any(type(x) not in (int,float) or not math.isfinite(x) for x in points[key]) for key in ("bend","arm_tip","link_pickup")):
            return _fail("Complete the bar bend, lever-arm tip and suspension pickup XYZ coordinates.")
        reflection=np.array([1.,-1. if pose["side"]=="right" else 1.,1.])
        bend,tip0,p0=[np.asarray(points[key],dtype=float)*reflection for key in ("bend","arm_tip","link_pickup")]
        if abs(bend[1])<1e-7:return _fail("Mirrored bar bends do not define a transverse shaft.")
        bearing=points.get("bearing")
        if isinstance(bearing,list) and len(bearing)==3 and all(type(x) in (int,float) and math.isfinite(x) for x in bearing):
            if math.hypot(bearing[0]-bend[0],bearing[2]-bend[2])>1e-3:return _fail("Entered bearing is not on the straight shaft through the bar bends.")
        previous=0.;result=None
        path=pose.get("_path") or [pose]
        for sample in path:
            pickup,_=_pickup(axle,sample,settings)
            result=_arm(bend,tip0,p0,pickup,previous);previous=result["angleRad"]
        return {"available":True,**result,"pathSamples":len(path),"axis":[0.,1.,0.]}
    except (KeyError,TypeError,ValueError,OverflowError,np.linalg.LinAlgError) as error:return _fail(str(error))


def arb_velocity(axle,pose,settings,velocity,arb):
    if not arb.get("available"):return _fail(arb.get("reason","ARB unavailable."))
    if not velocity.get("ok"):return _fail(velocity.get("reason","Suspension tangent unavailable."))
    try:
        pickup,rate=_pickup(axle,pose,settings,velocity)
        tip,bend=np.asarray(arb["tip"]),np.asarray(arb["bend"])
        s=tip-pickup;tangent=np.cross([0.,1.,0.],tip-bend)
        denominator=float(s@tangent)
        if abs(denominator)<1e-10*max(1.,np.linalg.norm(s)*np.linalg.norm(tangent)):return _fail("ARB derivative is singular at a toggle.")
        angular=float(s@rate/denominator)
        return {"available":True,"angleRateDeg":math.degrees(angular),"arcRateMm":arb["radiusMm"]*angular}
    except (KeyError,TypeError,ValueError,OverflowError,np.linalg.LinAlgError) as error:return _fail(str(error))
