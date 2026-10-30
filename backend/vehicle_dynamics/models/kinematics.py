"""Full-vehicle prescribed-travel channel assembly, calculated only in Python.

Definitions/derivation: suspension-studio/docs/KINEMATICS-DASHBOARD.md.
No force equilibrium, chassis attitude prediction or tire compliance is inferred.
"""
import math
import numpy as np

from .suspension import solve_corner, corner_velocity, actuator_velocity
from .steering import _metrics as nominal_ackermann_metrics
from .anti_effects import anti_effect
from .arb_kinematics import solve_arb, arb_velocity
from .kinematic_geometry import (
    instantaneous_screw_axis, circle_contact, kinematic_roll_center,
    kinematic_pitch_center, steering_geometry, front_view_instant_center,
    side_view_instant_center,
)

AXLES = ("front", "rear")
SIDES = ("left", "right")
RAD = math.pi / 180


def prescribed_bump(mode, value, station, reference):
    """Return bump mm and its exact derivative per displayed input unit."""
    if mode == "heave":
        return float(value), 1.0
    if mode == "steering":
        return 0.0, 0.0
    arm = -(station[0] - reference["x"]) if mode == "pitch" else station[1] - reference["y"]
    angle = value * RAD
    return float(arm * math.tan(angle)), float(arm * RAD / math.cos(angle)**2)


def alignment_rates(spindle, derivative, side):
    n, dn = np.asarray(spindle), np.asarray(derivative)
    horizontal2 = float(n[0]**2+n[1]**2)
    if horizontal2 < 1e-12:
        return None
    camber = float(dn[2]/math.sqrt(horizontal2)/RAD)
    steer = float((n[0]*dn[1]-n[1]*dn[0])/horizontal2/RAD)
    return {"camber":camber, "toe":(-1 if side == "right" else 1)*steer, "steer":steer}


def channel_registry(mode, design=None):
    """Every scalar is named by definition, not an ambiguous vendor nickname."""
    angular = mode in ("pitch", "roll")
    input_unit = "deg" if angular else "mm"
    channels = []
    corner_metrics = [
        ("bump", "Wheel-center bump", "mm", "Dimensions", "Upward wheel-center travel relative to chassis."),
        ("camber", "Camber", "deg", "Alignment", "Chassis-relative; positive top outward; entered static alignment included."),
        ("toe", "Toe-in", "deg", "Alignment", "Chassis-relative; positive inward on both sides."),
        ("steer", "Wheel heading", "deg", "Steering", "Positive toward the vehicle right; entered static alignment included."),
        ("deltaCamber", "Camber change from static", "deg", "Alignment", "Solved camber minus entered static camber."),
        ("deltaToe", "Toe-in change from static", "deg", "Alignment", "Solved toe-in minus entered static toe-in."),
        ("camberGain", "Camber gain per wheel bump", "deg/mm", "Alignment", "Local derivative at fixed rack; not its reciprocal."),
        ("toeGain", "Bump steer per wheel bump", "deg/mm", "Alignment", "Local toe-in derivative at fixed rack."),
        ("camberModeGain", "Camber sensitivity to sweep input", f"deg/{input_unit}", "Alignment", "Derivative with respect to the displayed mode input."),
        ("toeModeGain", "Toe-in sensitivity to sweep input", f"deg/{input_unit}", "Alignment", "Derivative with respect to the displayed mode input."),
        ("compression", "Damper compression", "mm", "Actuation", "Static eye-to-eye length minus current length."),
        ("damperLength", "Damper eye-to-eye length", "mm", "Actuation", "Ideal pin-to-pin telescoping length."),
        ("motionRatio", "Motion ratio · damper / wheel", "mm/mm", "Actuation", "Instantaneous dc/db; positive shortening per upward wheel bump, at fixed rack."),
        ("compressionModeGain", "Damper sensitivity to sweep input", f"mm/{input_unit}", "Actuation", "dc/dq; different from local damper/wheel ratio in pitch, roll or steering."),
        ("rockerAngle", "Rocker rotation", "deg", "Actuation", "Right-hand rotation about the explicitly entered reflected rocker direction."),
        ("caster", "Caster", "deg", "Steering", "Positive when upper steering-axis point is rearward of lower."),
        ("kpi", "Kingpin inclination", "deg", "Steering", "Positive upper axis inward; SAE side conversion explicit."),
        ("trail", "Mechanical trail", "mm", "Steering", "Steering-axis intersection ahead of reference contact, measured along wheel heading."),
        ("scrub", "Scrub radius", "mm", "Steering", "Contact outward of steering-axis intersection, along wheel lateral direction."),
        ("screwPitch", "Suspension instantaneous screw pitch", "mm/rad", "Centers", "Parallel velocity divided by angular speed; zero only for instantaneous pure rotation."),
        ("residual", "Constraint closure residual", "mm", "Diagnostics", "Maximum accepted physical closure residual."),
        ("condition", "Scaled constraint condition number", "1", "Diagnostics", "Translation/rotation scaling uses a 300 mm characteristic length."),
        ("brakeAnti", "Braking anti · front dive / rear lift", "%", "Anti-effects", "Geometric fraction of nominal longitudinal load transfer; requires CG height, braking fraction and torque reaction."),
        ("driveAnti", "Drive anti · front lift / rear squat", "%", "Anti-effects", "Geometric fraction of nominal longitudinal load transfer; requires CG height, drive fraction and torque reaction."),
        ("arbAngle", "U-bar lever-arm angle", "deg", "Anti-roll bar", "Signed about global +Y for both arms of the explicitly selected straight transverse bar."),
        ("arbArc", "U-bar lever-tip arc displacement", "mm", "Anti-roll bar", "Lever perpendicular radius times its rotation; not a guessed vertical drop-link displacement."),
        ("arbAngleModeGain", "U-bar angle sensitivity to sweep input", f"deg/{input_unit}", "Anti-roll bar", "d(arm angle)/dq; numerator and denominator explicitly stated."),
        ("arbArcModeGain", "U-bar arc sensitivity to sweep input", f"mm/{input_unit}", "Anti-roll bar", "d(tip arc displacement)/dq for the selected mode."),
    ]
    for coord in "XYZ":
        corner_metrics.extend([
            (f"wheel{coord}", f"Wheel-center {coord}", "mm", "Dimensions", "Front-origin chassis coordinates."),
            (f"contact{coord}", f"Ideal tire support point {coord}", "mm", "Dimensions", "Lowest point of the rigid tire center-plane circle; moves with rig travel."),
        ])
    for view in ("front", "side"):
        for coord in "XYZ":
            corner_metrics.append((f"{view}IC{coord}",f"{view.title()}-view instant center {coord}","mm","Centers","Spatial instantaneous-axis intersection with the wheel-center transverse/longitudinal plane; may be infinite or undefined."))
    for axle in AXLES:
        for side in SIDES:
            entity=f"{axle}.{side}"
            for metric,title,unit,group,description in corner_metrics:
                channels.append(dict(id=f"{entity}.{metric}",plot=metric,title=title,label=f"{axle.title()} {side}",unit=unit,group=group,entity=entity,description=description))
            if design:
                for key in design["axles"][axle]["hardpoints"]:
                    label=key.replace("_front","_fore").replace("_rear","_aft").replace("_"," ")
                    for coord in "XYZ":
                        channels.append(dict(id=f"{entity}.point_{key}_{coord}",plot=f"point_{key}_{coord}",title=f"{label.title()} {coord}",label=f"{axle.title()} {side}",unit="mm",group="Hardpoints",entity=entity,description="Solved front-origin chassis coordinates. Unresolved rocker markers are omitted."))
        for metric,title,unit,group,description in [
            ("rcX","Kinematic roll-center X","mm","Centers","Front-origin chassis X; spatial instantaneous-axis construction."),
            ("rcY","Kinematic roll-center lateral migration","mm","Centers","Chassis Y; positive right."),
            ("rcZ","Kinematic roll-center Z","mm","Centers","Chassis Z; positive downward."),
            ("rcHeight","Kinematic roll-center height","mm","Centers","Mean ideal tire support-point Z minus center Z; not ground height on a flat-road model."),
            ("track","Wheel-center track","mm","Dimensions","Right wheel-center Y minus left wheel-center Y."),
            ("trackChange","Track change from static","mm","Dimensions","Current projected track minus entered static track."),
            ("totalToe","Total toe-in","deg","Alignment","Sum of left and right inward-positive toe."),
            ("arbTwist","U-bar twist","deg","Anti-roll bar","Right arm angle minus left arm angle about the same global +Y shaft direction; no stiffness/torque model."),
            ("arbTwistModeGain","U-bar twist sensitivity to input",f"deg/{input_unit}","Anti-roll bar","Derivative of right-minus-left arm twist with respect to the sweep input."),
        ]:
            channels.append(dict(id=f"{axle}.{metric}",plot=metric,title=title,label=f"{axle.title()} axle",unit=unit,group=group,entity=axle,description=description))
    for side in SIDES:
        for metric,title,unit in [("pitchCenterX","Kinematic pitch-center X","mm"),("pitchCenterY","Kinematic pitch-center Y","mm"),("pitchCenterZ","Kinematic pitch-center Z","mm"),("wheelbase","Wheel-center wheelbase","mm")]:
            channels.append(dict(id=f"{side}.{metric}",plot=metric,title=title,label=f"{side.title()} side",unit=unit,group="Dimensions" if metric=="wheelbase" else "Centers",entity=f"vehicle.{side}",description="Front-origin chassis frame; projected X wheelbase or instantaneous-axis center construction."))
    channels.append(dict(id="vehicle.rollAxis",plot="rollAxis",title="Roll-axis inclination",label="Vehicle",unit="deg",group="Centers",entity="vehicle",description="atan2(rear center Z − front center Z, front center X − rear center X); positive axis rises forward."))
    for metric,title,unit,description in [
        ("insideDeg","Inside wheel heading magnitude","deg","Positive heading magnitude when both front wheels turn the same way."),
        ("outsideDeg","Outside wheel heading magnitude","deg","Positive heading magnitude when both front wheels turn the same way."),
        ("idealOutsideDeg","Ideal Ackermann outside heading","deg","Nominal fixed-wheelbase/track, zero-slip reference; not the migrating contact-point ICR."),
        ("ackermannErrorDeg","Ackermann outside-angle error","deg","Actual outside heading magnitude minus ideal outside heading; not a percentage."),
        ("ackermannPercent","Ackermann · cotangent convention","%","100 L/T (cot(outside) − cot(inside)); uses entered wheelbase/track; unavailable at straight ahead. Distinct from the OptimumG inside-angle ratio."),
    ]:
        channels.append(dict(id=f"front.{metric}",plot=metric,title=title,label="Front axle",unit=unit,group="Steering",entity="front",description=description))
    return channels


def _complete(vector):
    return isinstance(vector, list) and len(vector)==3 and all(type(x) in (int,float) and math.isfinite(x) for x in vector)


def _aligned(settings):
    return all(type(settings.get(k)) in (int,float) for k in ("staticCamberDeg","staticToeInDeg"))


def _damper_length(axle, pose):
    p=pose["points"]
    first,second=("strut_top","strut_bottom") if axle["topology"]=="macpherson" else ("damper_top","actuation_outer" if axle["actuation"]=="direct" else "rocker_damper")
    return float(np.linalg.norm(np.asarray(p[first])-p[second]))


def _center(construction, axis_a, contact_a, axis_b, contact_b):
    for item in (contact_a,contact_b,axis_a,axis_b):
        if not item or not item.get("available"):
            return {"available":False,"reason":item.get("reason","An accepted corner is required.") if item else "An accepted corner is required."}
    return construction(axis_a,contact_a["point"],axis_b,contact_b["point"])


def sweep_kinematics(design, sweep):
    mode=sweep["mode"]
    channels=channel_registry(mode,design)
    settings=design["kinematics"]
    rows=[]
    for value in np.linspace(sweep["min"],sweep["max"],sweep["samples"]):
        row={"input":float(value),"values":{c["id"]:None for c in channels},"unavailable":{},"poses":{},"diagnostics":{"maxResidualMm":None,"maxConditionNumber":None}}
        axes,contacts,centers={},{},{}
        residuals,conditions=[],[]
        def put(prefix, metric, number, reason="Calculation unavailable at this position."):
            key=f"{prefix}.{metric}"
            if number is not None and math.isfinite(float(number)):
                row["values"][key]=float(number)
            else:
                row["unavailable"][key]=reason
        for name in AXLES:
            axle=design["axles"][name]; alignment=settings["axles"][name]
            entered=_aligned(alignment)
            static={"camberDeg":alignment["staticCamberDeg"],"toeInDeg":alignment["staticToeInDeg"]} if entered else None
            rocker=alignment["rockerAxis"] if _complete(alignment["rockerAxis"]) else None
            offset=np.array([-design["wheelbase"] if name=="rear" else 0.,0.,0.])
            pair={}
            for side in SIDES:
                entity=f"{name}.{side}"
                station=np.asarray(axle["hardpoints"]["wheel_center"])*[1,-1 if side=="right" else 1,1]+offset
                bump,rate=prescribed_bump(mode,float(value),station,settings["reference"])
                rack=float(value) if mode=="steering" and axle["steered"] else 0.
                pose=solve_corner(axle,bump,rack,side=side,static_alignment=static,rocker_axis=rocker,trace=alignment.get("arbModel")=="straight-transverse" and axle.get("antiRollBar",{}).get("enabled") is True)
                pair[side]=pose
                if not pose["ok"]:
                    for c in channels:
                        if c["entity"]==entity: row["unavailable"][c["id"]]=pose["reason"]
                    continue
                p=pose["points"]; center=np.asarray(p["wheel_center"])+offset
                residuals.append(pose["error"]);conditions.append(pose["diagnostics"]["conditionNumber"])
                put(entity,"bump",bump);put(entity,"residual",pose["error"]);put(entity,"condition",conditions[-1])
                for index,coord in enumerate("XYZ"):put(entity,f"wheel{coord}",center[index])
                for key,point in p.items():
                    solved=not(key in ("rocker_rod","rocker_damper") and not pose["actuator"]["ok"])
                    for index,coord in enumerate("XYZ"):
                        put(entity,f"point_{key}_{coord}",point[index]+offset[index] if solved else None,pose["actuator"].get("reason","Actuator unavailable."))
                local=corner_velocity(axle,pose)
                motion=corner_velocity(axle,pose,bump_rate=rate,rack_rate=1. if mode=="steering" else 0.)
                arb=solve_arb(axle,pose,alignment)
                arb_rate=arb_velocity(axle,pose,alignment,motion,arb)
                pose["arb"]={**arb,"modeAngleRateDeg":arb_rate.get("angleRateDeg")}
                pose.pop("_path",None)
                for metric,number in (("arbAngle",arb.get("angleDeg")),("arbArc",arb.get("arcMm")),("arbAngleModeGain",arb_rate.get("angleRateDeg")),("arbArcModeGain",arb_rate.get("arcRateMm"))):
                    put(entity,metric,number,arb_rate.get("reason",arb.get("reason","ARB unavailable.")))
                unknown="Enter this axle's static camber and toe-in; neither is inferred from hardpoints."
                if entered:
                    for metric in ("camber","toe","steer"):put(entity,metric,pose[metric])
                    put(entity,"deltaCamber",pose["camber"]-alignment["staticCamberDeg"])
                    put(entity,"deltaToe",pose["toe"]-alignment["staticToeInDeg"])
                for velocity,suffix in ((local,"Gain"),(motion,"ModeGain")):
                    rates=alignment_rates(pose["spindle"],velocity["spindle"],side) if entered and velocity["ok"] else None
                    for metric in ("camber","toe"):put(entity,metric+suffix,rates[metric] if rates else None,unknown if not entered else velocity.get("reason","Projected spindle derivative is singular."))
                if not entered:
                    for metric in ("camber","toe","steer","deltaCamber","deltaToe"):put(entity,metric,None,unknown)
                actuator=pose["actuator"]
                put(entity,"compression",pose["compression"],actuator.get("reason","Actuator unavailable."))
                put(entity,"damperLength",_damper_length(axle,pose) if actuator["ok"] else None,actuator.get("reason","Actuator unavailable."))
                put(entity,"rockerAngle",actuator.get("angleRad")/RAD if actuator.get("angleRad") is not None else None,actuator.get("reason","Direct or strut actuation has no rocker angle."))
                for velocity,metric in ((local,"motionRatio"),(motion,"compressionModeGain")):
                    derivative=actuator_velocity(axle,pose,velocity,rocker_axis=rocker)
                    put(entity,metric,derivative.get("compressionRate") if derivative["ok"] else None,derivative.get("reason","Actuator derivative unavailable."))
                axis=instantaneous_screw_axis(center,local["linear"],local["angular"]) if local["ok"] else {"available":False,"reason":local["reason"]}
                axes[entity]=axis
                put(entity,"screwPitch",axis.get("pitchMmPerRad") if axis["available"] else None,axis.get("reason","Instantaneous axis unavailable."))
                for view,construction in (("front",front_view_instant_center),("side",side_view_instant_center)):
                    ic=construction(axis,center)
                    for index,coord in enumerate("XYZ"):put(entity,f"{view}IC{coord}",ic["point"][index] if ic["available"] else None,ic.get("reason","Instant center unavailable."))
                contact=circle_contact(center,pose["spindle"],design["wheelDiameter"]/2) if entered else {"available":False,"reason":unknown}
                contacts[entity]=contact
                force=settings.get("forceReference",{})
                for kind,share in (("brake","frontBrakeFraction"),("drive","frontDriveFraction")):
                    fraction=force.get(share)
                    if fraction is not None and name=="rear":fraction=1-fraction
                    anti=anti_effect(center,contact["point"],pose["spindle"],local,axle=name,wheelbase=design["wheelbase"],cg_height=force.get("cgHeightMm"),axle_fraction=fraction,reaction=alignment.get(kind+"Reaction")) if contact["available"] else {"available":False,"reason":unknown}
                    put(entity,kind+"Anti",anti.get("percent"),anti.get("reason","Anti-effect unavailable."))
                for index,coord in enumerate("XYZ"):put(entity,f"contact{coord}",contact["point"][index] if contact["available"] else None,contact.get("reason",unknown))
                if contact["available"] and axle["topology"]!="multi-link":
                    lower=np.asarray(p["lca_outer"])+offset
                    upper=np.asarray(p["strut_top"] if axle["topology"]=="macpherson" else p["uca_outer"])+offset
                    steering=steering_geometry(lower,upper,contact["point"],pose["spindle"],side)
                    for metric,source in (("caster","casterDeg"),("kpi","kingpinInclinationDeg"),("trail","mechanicalTrailMm"),("scrub","scrubRadiusMm")):
                        put(entity,metric,steering.get(source),steering.get("reason","Steering construction unavailable."))
                else:
                    for metric in ("caster","kpi","trail","scrub"):put(entity,metric,None,unknown if not entered else "A virtual steering-axis model is required for this multi-link topology.")
            pair["ok"]=pair["left"]["ok"] and pair["right"]["ok"]
            row["poses"][name]=pair
            if pair["ok"]:
                track=pair["right"]["points"]["wheel_center"][1]-pair["left"]["points"]["wheel_center"][1]
                put(name,"track",track);put(name,"trackChange",track-axle["track"])
                put(name,"totalToe",pair["left"]["toe"]+pair["right"]["toe"] if entered else None,"Enter this axle's static alignment.")
                la,ra=pair["left"]["arb"],pair["right"]["arb"]
                put(name,"arbTwist",ra["angleDeg"]-la["angleDeg"] if la["available"] and ra["available"] else None,"Both U-bar arms must have a regular, fully defined solution.")
                lr,rr=la.get("modeAngleRateDeg"),ra.get("modeAngleRateDeg")
                put(name,"arbTwistModeGain",rr-lr if rr is not None and lr is not None else None,"Both U-bar arm derivatives must be available.")
            l,r=f"{name}.left",f"{name}.right"
            center_result=_center(kinematic_roll_center,axes.get(l),contacts.get(l),axes.get(r),contacts.get(r))
            centers[name]=center_result
            for i,coord in enumerate("XYZ"):put(name,f"rc{coord}",center_result["point"][i] if center_result["available"] else None,center_result.get("reason","Roll center unavailable."))
            height=(contacts[l]["point"][2]+contacts[r]["point"][2])/2-center_result["point"][2] if center_result["available"] else None
            put(name,"rcHeight",height,center_result.get("reason","Roll center unavailable."))
        for side in SIDES:
            f,r=f"front.{side}",f"rear.{side}"
            pc=_center(kinematic_pitch_center,axes.get(f),contacts.get(f),axes.get(r),contacts.get(r))
            for i,coord in enumerate("XYZ"):put(side,f"pitchCenter{coord}",pc["point"][i] if pc["available"] else None,pc.get("reason","Pitch center unavailable."))
            fp,rp=row["poses"]["front"][side],row["poses"]["rear"][side]
            put(side,"wheelbase",fp["points"]["wheel_center"][0]-rp["points"]["wheel_center"][0]+design["wheelbase"] if fp["ok"] and rp["ok"] else None,"A corner has no accepted pose.")
        f,r=centers["front"],centers["rear"]
        inclination=math.atan2(r["point"][2]-f["point"][2],f["point"][0]-r["point"][0])/RAD if f["available"] and r["available"] else None
        put("vehicle","rollAxis",inclination,"Both kinematic roll centers must be finite.")
        front=row["poses"]["front"]
        ack={}
        if front["ok"] and _aligned(settings["axles"]["front"]) and not design["axles"]["rear"]["steered"]:
            toe=settings["axles"]["front"]["staticToeInDeg"]
            ack=nominal_ackermann_metrics({"wheelbase":design["wheelbase"],"frontTrack":design["axles"]["front"]["track"],"left":{"staticToeDeg":toe},"right":{"staticToeDeg":toe}},
                {"headingDeg":front["left"]["steer"],"toeInDeg":front["left"]["toe"]},
                {"headingDeg":front["right"]["steer"],"toeInDeg":front["right"]["toe"]})
        for metric in ("insideDeg","outsideDeg","idealOutsideDeg","ackermannErrorDeg","ackermannPercent"):
            put("front",metric,ack.get(metric),"Requires solved front alignment, a nonsteered rear axle and both front wheels turning the same way; percentage is undefined near straight ahead.")
        row["diagnostics"]={"maxResidualMm":max(residuals) if residuals else None,"maxConditionNumber":max(conditions) if conditions else None}
        for c in channels:
            if row["values"][c["id"]] is None:row["unavailable"].setdefault(c["id"],"Required corner pose or geometric construction is unavailable.")
        rows.append(row)
    return dict(model="rigid_suspension_v1",coordinate_system="SAE_J670_Z_DOWN",hardpoint_frame="AXLE_LOCAL",length_unit="mm",angle_unit="deg",mode=mode,input_unit="deg" if mode in ("pitch","roll") else "mm",channels=channels,rows=rows,
        assumptions=["Rigid links, spherical outboard joints and rigid uprights; no compliance, loads or tire forces.","Chassis-fixed rig: b = h − (X − Xref) tan(pitch) + (Y − Yref) tan(roll), evaluated at static front-origin wheel stations.","Chassis-relative camber/toe require entered static alignment. Per-axle left geometry and alignment are mirrored to the right.","Motion ratio is instantaneous damper shortening / upward wheel bump, at fixed rack.","Ideal rigid circular tire center-plane support points follow the wheels; they need not lie on a common flat road.","Kinematic centers use spatial instantaneous-axis planes; they are not assumed chassis rotation centers or force-based centers."],
        coverage=[dict(family="Alignment, actuation, travel, dimensions and sensitivities",status="available",reason="Calculated where the required measured inputs and a regular assembly are available."),dict(family="Spatial kinematic roll/pitch centers",status="available",reason="Requires static alignment; parallel planes and unresolved instantaneous axes remain unavailable."),dict(family="Anti-dive / anti-squat / anti-lift",status="available",reason="Spatial virtual-work coefficients with explicit CG, force splits and torque paths. Nominal load-transfer reference, not predicted body attitude."),dict(family="Anti-roll-bar motion ratios and twist",status="available",reason="Explicit straight transverse U-bar model, pickup carrier and complete three-point geometry required. Bearing, when supplied, must be coaxial. No stiffness or forces inferred."),dict(family="Joint forces, moments and compliance",status="unavailable",reason="Outside this rigid kinematic model; cannot be derived from travel inputs alone.")],
        warnings=["Numerical verification is not experimental vehicle validation.","The documented channel families are not a claim of complete current OptimumKinematics equivalence."])
