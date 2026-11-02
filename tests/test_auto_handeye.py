import copy
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from experiments.panthera_ht.auto_handeye_core import poses, solve

class AutoHandeyeTests(unittest.TestCase):
    def fixture(self):
        rng=np.random.default_rng(20261022)
        x=np.eye(4);x[:3,:3]=Rotation.from_euler('xyz',[-109,-2,-86],degrees=True).as_matrix();x[:3,3]=[-.089,.005,.064]
        board=np.eye(4);board[:3,3]=[.5,.1,.3];samples=[]
        for i in range(15):
            h=np.eye(4);h[:3,:3]=Rotation.from_rotvec(rng.normal(0,.25,3)).as_matrix();h[:3,3]=rng.normal(0,.03,3)
            v=np.linalg.inv(x)@np.linalg.inv(h)@board
            samples.append({'role':'validation' if i in (4,9,14) else 'train','T_base_tcp':h.tolist(),'T_camera_marker':v.tolist()})
        return x,samples
    def test_known_transform_recovered_with_holdout(self):
        x,samples=self.fixture();r=solve(samples)
        np.testing.assert_allclose(r['T_tcp_camera'],x,atol=1e-7)
        self.assertTrue(r['validation']['passed']);self.assertEqual(r['train_count'],12);self.assertEqual(r['validation_count'],3)
    def test_validation_never_changes_fit(self):
        _,samples=self.fixture();good=solve(samples)
        for s in samples:
            if s['role']=='validation':s['T_camera_marker'][0][3]+=.03
        bad=solve(samples);np.testing.assert_allclose(bad['T_tcp_camera'],good['T_tcp_camera'],atol=1e-9)
        self.assertFalse(bad['validation']['passed'])
    def test_plan_has_15_distinct_poses_and_3_holdouts(self):
        q=[0.,.6,.6,0.,.3,0.];lo=[-2.4,0,0,-1.6,-1.7,-2.5];hi=[2.4,3.2,4,1.6,1.7,2.5]
        p=poses(q,lo,hi);self.assertEqual(len(p),15);self.assertEqual(sum(x['role']=='validation' for x in p),3)
        self.assertEqual(len({tuple(x['q_rad']) for x in p}),15)
        for row in p:np.testing.assert_allclose(row['q_rad'][:3],q[:3])
    def test_envelope_does_not_expand_limits(self):
        with self.assertRaises(ValueError):poses([0,.6,.6,1.59,.3,0],[-2.4,0,0,-1.6,-1.7,-2.5],[2.4,3.2,4,1.6,1.7,2.5])
    def test_degenerate_rotations_rejected(self):
        _,samples=self.fixture()
        for s in samples:
            h=np.array(s['T_base_tcp']);h[:3,:3]=np.eye(3);s['T_base_tcp']=h.tolist()
        with self.assertRaises(ValueError):solve(samples)

if __name__=='__main__':unittest.main()
