"""Spatial virtual-work anti coefficients; see KINEMATICS-ANTI-EFFECTS.md."""
import math
import numpy as np


def anti_effect(center, contact, spindle, velocity, *, axle, wheelbase,
                cg_height, axle_fraction, reaction):
    unavailable=lambda reason:{"available":False,"percent":None,"reason":reason}
    if not velocity or velocity.get("ok") is not True:
        return unavailable("A regular wheel-bump constraint tangent is required.")
    if type(cg_height) not in (int,float) or not math.isfinite(cg_height) or cg_height<=0:
        return unavailable("Enter the positive nominal CG height for anti-effect percentages.")
    if type(axle_fraction) not in (int,float) or not math.isfinite(axle_fraction) or not 0<=axle_fraction<=1:
        return unavailable("Enter the front brake/drive force fraction between 0 and 1.")
    if axle not in ("front","rear") or not isinstance(wheelbase,(int,float)) or not math.isfinite(wheelbase) or wheelbase<=0:
        return unavailable("A valid axle and nominal wheelbase are required.")
    if axle_fraction==0:
        return {"available":True,"percent":0.,"coefficient":0.,"reaction":reaction}
    if reaction not in ("upright","chassis"):
        return unavailable("Select where this brake/drive spin torque is reacted: upright or chassis.")
    try:
        c,q,n,v,w=[np.asarray(value,dtype=float) for value in (center,contact,spindle,velocity["linear"],velocity["angular"])]
        if any(x.shape!=(3,) or not np.isfinite(x).all() for x in (c,q,n,v,w)) or np.linalg.norm(n)<1e-9:
            return unavailable("Finite center, contact, spindle and tangent vectors are required.")
        if abs(v[2]+1)>1e-7:
            return unavailable("Anti coefficients require tangent normalization to one millimeter upward wheel bump.")
        n=n/np.linalg.norm(n)
        moment=np.cross(q-c,[1.,0.,0.])
        if reaction=="chassis":moment-=n*float(n@moment)
        coefficient=float(v[0]+moment@w)
        value=(1 if axle=="front" else -1)*100*axle_fraction*wheelbase/cg_height*coefficient
        if not math.isfinite(value):return unavailable("Anti-effect normalization exceeds the finite numerical range.")
        return {"available":True,"percent":float(value),"coefficient":coefficient,"reaction":reaction}
    except (TypeError,ValueError,KeyError,OverflowError) as error:
        return unavailable(str(error))
