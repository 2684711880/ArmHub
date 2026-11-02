"""Real SDK/Pinocchio import orders, with no Robot construction or connections."""
from pathlib import Path
import subprocess
import sys
import unittest


class KinematicsImportTests(unittest.TestCase):
    def test_official_variants_match_tcp_and_saved_prepare(self):
        script='''
import json
from pathlib import Path
import numpy as np
{first}
from armhub.adapters.panthera_kinematics import PantheraKinematics
c=json.loads(Path('robots/panthera_ht/hardware/config.json').read_text())
k=PantheraKinematics(c['lower_rad'],c['upper_rad'])
prepared=json.loads(Path('integrations/panthera_lerobot/configs/teaching.json').read_text())['prepared']
for q in ([0,.8,1.,0.,0.,0.],prepared['q']):
    actual=k.fk(q)
    full=k.np.zeros(k.model.nq)
    for name,value in zip(k.sdk.joint_names,q):
        j=k.model.joints[k.model.getJointId(name)];full[j.idx_q]=value
    k.pin.forwardKinematics(k.model,k.data,full)
    k.pin.updateFramePlacements(k.model,k.data)
    reference=k.data.oMf[k.model.getFrameId('gripper_center')]
    np.testing.assert_allclose(actual['position_m'],reference.translation,atol=1e-10)
    np.testing.assert_allclose(k.Rotation.from_quat(actual['quaternion_xyzw']).as_matrix(),reference.rotation,atol=1e-10)
np.testing.assert_allclose(k.fk(prepared['q'])['position_m'],prepared['position_m'],atol=1e-8)
k.check_path(prepared['q'],prepared['q'])
print('PASS real FK, saved pose and collision check; no devices')
'''
        root=Path(__file__).resolve().parents[1]
        for first in ('','import lerobot_robot_panthera.panthera_follower'):
            with self.subTest(lerobot_first=bool(first)):
                result=subprocess.run([sys.executable,'-c',script.format(first=first)],
                    cwd=root,capture_output=True,text=True,timeout=45)
                self.assertEqual(result.returncode,0,result.stdout+result.stderr)
                self.assertIn('PASS real FK',result.stdout)


if __name__=='__main__':unittest.main()
