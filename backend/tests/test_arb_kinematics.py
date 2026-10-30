from copy import deepcopy
import math
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from vehicle_dynamics.models.arb_kinematics import solve_arb, arb_velocity


class ArbKinematicsTests(unittest.TestCase):
    def setUp(self):
        self.axle={"topology":"double-wishbone","hardpoints":{"wheel_center":[0,-600,-250]},"antiRollBar":{"enabled":True,"type":"u-bar","points":{"bend":[-80,-250,-400],"arm_tip":[60,-250,-400],"link_pickup":[60,-500,-200],"bearing":[None,None,None]}}}
        self.settings={"arbModel":"straight-transverse","arbAttachment":"upright"}

    def pose(self,translation=(0,0,0),side="left"):
        reflection=np.array([1,-1 if side=="right" else 1,1])
        center=np.asarray(self.axle["hardpoints"]["wheel_center"])*reflection+translation
        return {"ok":True,"side":side,"rotation":[0,0,0],"points":{"wheel_center":center.tolist()},"actuator":{"ok":False}}

    def test_independent_known_arm_angle_and_link_length(self):
        a=math.radians(15);arm=np.array([140.,0,0]);shift=Rotation.from_rotvec([0,a,0]).apply(arm)-arm
        pose=self.pose(shift);pose["_path"]=[self.pose(),pose.copy()]
        result=solve_arb(self.axle,pose,self.settings)
        self.assertTrue(result["available"],result)
        self.assertAlmostEqual(result["angleDeg"],15,places=10)
        self.assertAlmostEqual(result["arcMm"],140*a,places=10)
        self.assertLess(result["residualMm"],1e-10)
        self.assertAlmostEqual(np.linalg.norm(np.array(result["tip"])-result["pickup"]),math.hypot(250,200),places=10)

    def test_analytic_derivative_matches_prescribed_rotation_and_finite_difference(self):
        a=.12;arm=np.array([140.,0,0]);rotated=Rotation.from_rotvec([0,a,0]).apply(arm);shift=rotated-arm
        pose=self.pose(shift);result=solve_arb(self.axle,pose,self.settings)
        velocity={"ok":True,"linear":np.cross([0,1,0],rotated).tolist(),"angular":[0,0,0]}
        derivative=arb_velocity(self.axle,pose,self.settings,velocity,result)
        self.assertAlmostEqual(derivative["angleRateDeg"],180/math.pi,places=9)
        self.assertAlmostEqual(derivative["arcRateMm"],140,places=9)
        h=1e-5
        sides=[solve_arb(self.axle,self.pose(Rotation.from_rotvec([0,a+sign*h,0]).apply(arm)-arm),self.settings)["angleDeg"] for sign in (-1,1)]
        self.assertAlmostEqual((sides[1]-sides[0])/(2*h),derivative["angleRateDeg"],delta=1e-7)

    def test_same_global_axis_yields_zero_symmetric_heave_twist(self):
        results=[solve_arb(self.axle,self.pose((0,0,-10),side),self.settings) for side in ("left","right")]
        self.assertTrue(all(x["available"] for x in results))
        self.assertAlmostEqual(results[0]["angleDeg"],results[1]["angleDeg"],places=10)

    def test_missing_carrier_model_geometry_and_incompatible_bearing(self):
        for key in ("arbAttachment","arbModel"):
            settings=deepcopy(self.settings);settings[key]=None
            self.assertFalse(solve_arb(self.axle,self.pose(),settings)["available"])
        self.axle["antiRollBar"]["points"]["bearing"]=[-70,-200,-400]
        self.assertFalse(solve_arb(self.axle,self.pose(),self.settings)["available"])
        self.axle["antiRollBar"]["points"]["bearing"]=[-80,-200,-400]
        self.assertTrue(solve_arb(self.axle,self.pose(),self.settings)["available"])
        self.axle["antiRollBar"]["points"]["bend"][1]=None
        self.assertFalse(solve_arb(self.axle,self.pose(),self.settings)["available"])

    def test_intervening_invalid_pose_does_not_become_valid_at_endpoint(self):
        pose=self.pose();pose["_path"]=[self.pose(),self.pose((0,0,-1000)),self.pose()]
        self.assertFalse(solve_arb(self.axle,pose,self.settings)["available"])


if __name__=="__main__":unittest.main()
