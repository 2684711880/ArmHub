import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from apps.handeye_rviz.kinematics import ArmFK,quaternion

class ManualHandeyeFKTests(unittest.TestCase):
    def test_fk_matches_official_tcp(self):
        from armhub.adapters.panthera_kinematics import PantheraKinematics
        m=PantheraKinematics([-2.4,0,0,-1.6,-1.7,-2.5],[2.4,3.2,4,1.6,1.7,2.5]);fk=ArmFK()
        for q in ([0,.6,.6,0,.3,0],[.2,1.,1.,-.4,.5,.3]):
            t=fk.transform(dict(zip([f'joint{i}' for i in range(1,7)],q)));official=m.fk(q)
            np.testing.assert_allclose(t[:3,3],official['position_m'],atol=1e-10)
            np.testing.assert_allclose(Rotation.from_quat(quaternion(t[:3,:3])).as_matrix(),Rotation.from_quat(official['quaternion_xyzw']).as_matrix(),atol=1e-10)

if __name__=='__main__':unittest.main()
