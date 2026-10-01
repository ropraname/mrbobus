import unittest
import numpy as np
from mrbobus_console.ground import separate_ground,floor_clearing_ranges,traversable_lawn_mask

class GroundTest(unittest.TestCase):
    def test_tilted_floor_and_low_box(self):
        rng=np.random.default_rng(8);xy=rng.uniform([.35,-1.2],[2.4,1.2],(1600,2));z=.02*xy[:,0]-.05*xy[:,1]-.1+rng.normal(0,.002,len(xy));floor=np.c_[xy,z]
        xy=rng.uniform([.4,-.12],[.65,.12],(70,2));box=np.c_[xy,.02*xy[:,0]-.05*xy[:,1]-.1+.07]
        obs,stats=separate_ground(np.r_[floor,box]);self.assertEqual(len(obs),70);self.assertGreater(stats['inliers'],1400)
    def test_missing_floor_rejected(self):
        with self.assertRaises(ValueError):separate_ground(np.zeros((50,3)))
    def test_chassis_excluded(self):
        rng=np.random.default_rng(2);xy=rng.uniform(-1.5,1.5,(2000,2));floor=np.c_[xy,np.full(len(xy),-.1)];body=np.array([[.1,.1,0.]])
        obs,_=separate_ground(np.r_[floor,body]);self.assertEqual(len(obs),0)

    def test_side_echo_mask_preserves_forward_box(self):
        rng=np.random.default_rng(4);xy=rng.uniform(-1.5,1.5,(2000,2));floor=np.c_[xy,np.full(len(xy),-.1)]
        samples=np.array([[.114,.245,0.],[.114,-.245,0.],[.45,0.,-.03],[.46,.01,-.03],[.47,-.01,-.03]])
        obs,_=separate_ground(np.r_[floor,samples]);self.assertEqual(len(obs),3);self.assertAlmostEqual(obs[0,0],.45)

    def test_isolated_low_outlier_rejected(self):
        rng=np.random.default_rng(5);xy=rng.uniform(-1.5,1.5,(2000,2));floor=np.c_[xy,np.full(len(xy),-.1)]
        obs,_=separate_ground(np.r_[floor,[[.25,0.,-.04]]]);self.assertEqual(len(obs),0)

    def test_floor_clear_does_not_extend_past_box_or_unknown(self):
        p=np.array([[1.,0.,-.1],[2.,0.,-.1]]);box=np.array([[.5,0.,-.03]])
        r=floor_clearing_ranges(p,[0,0,-.1],box);self.assertAlmostEqual(float(r[180]),.475,places=5);self.assertTrue(np.isinf(r[0]))

    def test_floor_history_never_replays_old_obstacles(self):
        rng=np.random.default_rng(9);xy=rng.uniform(-1.5,1.5,(2000,2));floor=np.c_[xy,np.full(len(xy),-.1)]
        current=np.array([[.5,0,.2]])
        obstacles,_=separate_ground(current,floor)
        self.assertEqual(len(obstacles),1);self.assertTrue(np.allclose(obstacles,current))

    def test_lawn_exception_is_low_and_inside_only(self):
        p=np.array([[.5,.5,.05],[.5,.5,.08],[1.5,.5,.05],[.5,.5,-.1]])
        mask=traversable_lawn_mask(p,[0,0,0],np.eye(4),(0,1,0,1))
        self.assertEqual(mask.tolist(),[True,False,False,False])
    def test_step_clearing_still_stops_at_high_obstacle(self):
        p=np.array([[1.,0.,.05]]);obs=np.array([[.5,0.,.15]])
        r=floor_clearing_ranges(p,[0,0,0],obs,p)
        self.assertAlmostEqual(float(r[180]),.475,places=5)
