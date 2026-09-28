import unittest,tempfile,json,math
from pathlib import Path
import numpy as np
from mrbobus_console.field_map import FieldMap,pose_matrix,transform

class MapTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.root=Path(self.tmp.name)
        rng=np.random.default_rng(9);p=rng.uniform([-.8,-.8,.15],[.8,.8,1.2],(2500,3))
        np.savez(self.root/'field.npz',points=p,bounds=[[-2,-2],[2,2]],trajectory=[[0,0,0]])
        self.map=FieldMap(self.root)
    def tearDown(self):self.tmp.cleanup()
    def test_manual_alignment_tracks_relative_motion(self):
        cur=pose_matrix(.8,.3,.2);self.map.set_pose({'x':-.5,'y':.4,'yaw':1.},cur)
        p=self.map.status(cur,True)['pose'];self.assertAlmostEqual(p['x'],-.5);self.assertAlmostEqual(p['yaw'],1.)
        self.assertIsNone(self.map.status(cur,False)['pose'])
    def test_goals_persist_without_pose_or_motion(self):
        self.map.goal({'x':.2,'y':.3,'yaw':.4});self.assertEqual(len(FieldMap(self.root).goals),1);self.assertIsNone(self.map.alignment)
        with self.assertRaises(ValueError):self.map.goal({'x':math.nan,'y':0,'yaw':0})
        with self.assertRaises(ValueError):self.map.goal({'x':30,'y':0,'yaw':0})
    def test_refine_small_initial_error(self):
        true=pose_matrix(.05,.04,.02);cloud=transform(self.map.points,np.linalg.inv(true));self.map.set_pose({'x':.10,'y':.07,'yaw':.05},np.eye(4))
        result=self.map.refine(cloud,np.eye(4));self.assertGreater(result['quality']['overlap'],.9)
        actual=self.map.status(np.eye(4),True)['pose'];self.assertLess(math.hypot(actual['x']-.05,actual['y']-.04),.025)
