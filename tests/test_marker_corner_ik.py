"""Replay a marker target from a boundary joint pose without hardware."""
import json
from pathlib import Path
import unittest
import numpy as np
from armhub.adapters.panthera_kinematics import PantheraKinematics

ROOT=Path(__file__).resolve().parents[1]

class MarkerCornerIKTests(unittest.TestCase):
    def test_boundary_seed_recovers_reachable_marker_target(self):
        cfg=json.loads((ROOT/'robots/panthera_ht/hardware/config.json').read_text())
        m=PantheraKinematics(cfg['lower_rad'],cfg['upper_rad'])
        q=[-.0691150426864624,0.,-.00251327408477664,.025132741779088974,-.03707079589366913,-.05215043947100639]
        target=[.52935333,.08349047,.19144662]
        quat=m.fk(q)['quaternion_xyzw']
        result=m.solve(q,target,quat)
        self.assertLess(result['ik_position_error_m'],.001)
        self.assertLess(result['ik_orientation_error_rad'],.01)
        self.assertTrue(np.all(np.array(result['target_joints_rad'])>=m.lower))
        self.assertTrue(np.all(np.array(result['target_joints_rad'])<=m.upper))

if __name__=='__main__':unittest.main()
