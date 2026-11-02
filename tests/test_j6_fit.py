"""离线解算数学与训练/验证隔离测试，不连接硬件。"""
import copy
import json
import math
from pathlib import Path
import tempfile
import unittest

import numpy as np

from examples.fit_j6_gravity import solve,validate_data


def synthetic_poses(bias=0.):
    result=[]
    for group in range(3):
        for angle in [-30,-20,-10,0,10,20,30]:
            q=math.radians(angle)
            features=[9.81*math.cos(q)*math.cos(group*.1),-9.81*math.sin(q)*math.cos(group*.1),1.]
            for direction,sign in [('positive',1),('negative',-1)]:
                result.append({'group':group,'role':'validation' if group==2 else 'train',
                               'j3_offset_deg':group*10,'j6_offset_deg':angle,'direction':direction,
                               'regressor_hy_hz_bias_mean':features,'position_rad_mean':[0.]*5+[q],
                               'j6_residual_nm_mean':features[0]*.001+features[1]*.014+bias+sign*.05})
    return result


class FitTests(unittest.TestCase):
    def test_known_first_moments_and_directional_gap_are_recovered(self):
        result=solve(synthetic_poses())
        np.testing.assert_allclose(result['fits']['gravity_paired']['parameters'],[.001,.014],atol=1e-12)
        self.assertLess(result['fits']['gravity_paired']['validation_rmse_nm'],1e-12)
        self.assertAlmostEqual(result['directional_half_gap_nm']['mean'],.05)
        full=solve(synthetic_poses(bias=-.01))
        np.testing.assert_allclose(full['fits']['gravity_bias_direction_individual']['parameters'],[.001,.014,-.01,.05],atol=1e-12)

    def test_validation_torques_cannot_change_training_parameters(self):
        poses=synthetic_poses()
        baseline=solve(poses)
        changed=copy.deepcopy(poses)
        for p in changed:
            if p['role']=='validation': p['j6_residual_nm_mean']+=.3
        shifted=solve(changed)
        for name in baseline['fits']:
            np.testing.assert_allclose(baseline['fits'][name]['parameters'],shifted['fits'][name]['parameters'])
        self.assertGreater(shifted['fits']['gravity_paired']['validation_rmse_nm'],.29)

    def test_missing_direction_or_mixed_group_labels_are_rejected(self):
        with self.assertRaisesRegex(ValueError,'正反配对'):
            solve(synthetic_poses()[:-1])
        mixed=synthetic_poses()
        mixed[0]['role']='validation'
        with self.assertRaisesRegex(ValueError,'同一 J3 组'):
            solve(mixed)

    def test_incomplete_run_is_not_reported_as_successful_experiment(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp)
            (root/'metadata.json').write_text(json.dumps({'robot':'panthera_ht','source':'hardware'}))
            (root/'summary.json').write_text(json.dumps({'status':'failed','hold_stop_confirmed':True}))
            (root/'poses.jsonl').write_text('')
            with self.assertRaisesRegex(ValueError,'未完整完成'):
                validate_data(root)


if __name__=='__main__':
    unittest.main()
