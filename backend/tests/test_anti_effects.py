import math
import unittest
import numpy as np
from vehicle_dynamics.models.anti_effects import anti_effect


class AntiEffectsTests(unittest.TestCase):
    def calculate(self, *, axle="front", reaction="upright", fraction=.6, spindle=(0,-1,0), velocity=None, height=300):
        # Planar side-view pivot 1000 mm behind, 150 mm above contact.
        return anti_effect([0,0,-250],[0,0,0],spindle,
            velocity or {"ok":True,"linear":[-.1,0,-1],"angular":[0,.001,0]},
            axle=axle,wheelbase=1600,cg_height=height,axle_fraction=fraction,reaction=reaction)

    def test_front_outboard_brake_recovers_reference_moment_equation(self):
        result=self.calculate()
        self.assertTrue(result["available"])
        self.assertAlmostEqual(result["percent"],100*.6*1600/300*.150)

    def test_chassis_spin_reaction_recovers_wheel_center_planar_formula(self):
        result=self.calculate(reaction="chassis")
        self.assertAlmostEqual(result["percent"],100*.6*1600/300*(-.1))

    def test_rear_sign_and_force_share_are_explicit(self):
        rear=self.calculate(axle="rear",fraction=.4,velocity={"ok":True,"linear":[.1,0,-1],"angular":[0,-.001,0]})
        self.assertAlmostEqual(rear["percent"],100*.4*1600/300*.15)

    def test_inclined_spindle_retains_overturning_moment(self):
        n=[0,-math.cos(.2),math.sin(.2)]
        result=self.calculate(reaction="chassis",spindle=n)
        # Only wheel-spin moment is removed; bearing tilt moment remains.
        expected=-.1+.250*(1-math.cos(.2)**2)
        self.assertAlmostEqual(result["coefficient"],expected)
        self.assertNotAlmostEqual(result["coefficient"],-.1,places=5)

    def test_virtual_work_matches_rigid_point_velocity(self):
        n=np.array([.2,-.95,.1]);n/=np.linalg.norm(n)
        v=np.array([.13,.07,-1.]);w=np.array([.003,-.001,.002]);r=np.array([12.,-10.,240.])
        result=anti_effect([10,20,-240],np.array([10,20,-240])+r,n,{"ok":True,"linear":v.tolist(),"angular":w.tolist()},axle="front",wheelbase=1600,cg_height=300,axle_fraction=.7,reaction="upright")
        self.assertAlmostEqual(result["coefficient"],(v+np.cross(w,r))[0])

    def test_no_force_zero_and_missing_inputs_stay_unavailable(self):
        self.assertEqual(self.calculate(fraction=0,reaction=None)["percent"],0)
        for kwargs in ({"height":None},{"fraction":None},{"reaction":None},{"height":0},{"fraction":1.2}):
            result=self.calculate(**kwargs);self.assertFalse(result["available"]);self.assertIsNone(result["percent"])
        self.assertFalse(self.calculate(velocity={"ok":True,"linear":[0,0,1],"angular":[0,0,0]})["available"])

    def test_geometric_anti_is_not_clamped_at_100_percent(self):
        self.assertGreater(self.calculate(height=30)["percent"],100)


if __name__=="__main__":unittest.main()
