"""Position-triggered feedforward and real upstream packet seam; no devices."""
import math
from pathlib import Path
from types import SimpleNamespace
import tempfile
import time
import unittest
from unittest.mock import Mock
import numpy as np
from integrations.panthera_lerobot.gripper_assist import (
    AssistConfig, PositionGripAssist, PositionTargetLimiter, install_gripper_assist)
from integrations.panthera_lerobot.gripper_trace import install_gripper_trace


class AssistTests(unittest.TestCase):
    def state(self, q, counter, **kwargs):
        return SimpleNamespace(**dict(dict(ID=7,num=counter,fault=0,time=time.time(),
            position=q,velocity=0.,torque=0.),**kwargs))

    def activate(self, control):
        control.step(1.5,self.state(1.5,1),0.)
        return control.step(.8,self.state(.8,2),.1)

    def test_initial_closed_position_never_assists(self):
        control=PositionGripAssist(AssistConfig())
        for i in range(5):
            status=control.step(.025,self.state(.025,i+1),i*.05)
            self.assertEqual(status['feedforward_nm'],0.)
            self.assertFalse(control.active)

    def test_initial_target_uses_actual_opening_and_slews_recorded_jump(self):
        limit=PositionTargetLimiter(AssistConfig())
        self.assertEqual(limit.step(.025,1.6,0.),1.6)
        self.assertAlmostEqual(limit.step(.111,1.6,.05),1.585)
        self.assertAlmostEqual(limit.step(.381,1.585,.1),1.57)

    def test_feedback_jump_is_rejected_instead_of_sending_large_position_error(self):
        limit=PositionTargetLimiter(AssistConfig())
        limit.step(1.5,1.5,0.)
        with self.assertRaisesRegex(RuntimeError,'限速跟踪'):
            limit.step(.8,.8,.05)

    def test_near_position_after_opening_ramps_bounded_closing_torque(self):
        control=PositionGripAssist(AssistConfig())
        self.assertEqual(self.activate(control)['feedforward_nm'],0.)
        mid=control.step(.8,self.state(.8,3),.25)['feedforward_nm']
        end=control.step(.8,self.state(.8,4),.4)['feedforward_nm']
        self.assertAlmostEqual(mid,-.05)
        self.assertAlmostEqual(end,-.1)

    def test_far_or_fast_gripper_does_not_activate(self):
        control=PositionGripAssist(AssistConfig())
        control.step(1.5,self.state(1.5,1),0.)
        self.assertFalse(control.step(.8,self.state(1.2,2),.1)['feedforward_nm'])
        self.assertFalse(control.active)
        control.step(.8,self.state(.8,3,velocity=-.3),.2)
        self.assertFalse(control.active)

    def test_partial_opening_immediately_removes_torque(self):
        control=PositionGripAssist(AssistConfig());self.activate(control)
        control.step(.78,self.state(.78,3),.3)
        status=control.step(.9,self.state(.8,4),.4)
        self.assertEqual(status['feedforward_nm'],0.)
        self.assertFalse(control.active)
        control.step(.8,self.state(.8,5),.5)
        self.assertFalse(control.active)

    def test_closed_mechanical_end_removes_torque(self):
        control=PositionGripAssist(AssistConfig());self.activate(control)
        status=control.step(.08,self.state(.09,3),.2)
        self.assertEqual(status['feedforward_nm'],0.)
        self.assertFalse(control.active)

    def test_feedback_fault_stale_torque_counter_and_cycle_guards(self):
        cases=[dict(fault=1),dict(time=time.time()-1),dict(torque=1.01),dict(position=float('nan'))]
        for values in cases:
            control=PositionGripAssist(AssistConfig())
            sample=self.state(.8,1)
            for key,value in values.items():setattr(sample,key,value)
            with self.subTest(values=values),self.assertRaises(RuntimeError):control.step(.8,sample,0.)
        for sample,stamp in [(self.state(.8,2),.5),(self.state(.8,1),.1)]:
            control=PositionGripAssist(AssistConfig());control.step(1.5,self.state(1.5,1),0.)
            with self.assertRaises(RuntimeError):control.step(.8,sample,stamp)

    def test_config_rejects_unbounded_or_invalid_assist(self):
        for values in (dict(torque_nm=.31),dict(torque_nm=0),dict(torque_nm=math.nan),
                       dict(close_at_rad=1.4),dict(ramp_s=0)):
            with self.subTest(values=values),self.assertRaises(ValueError):AssistConfig(**values)

    def test_real_official_sender_preserves_six_axes_and_packet_mode(self):
        from lerobot_robot_panthera.panthera_follower import PantheraFollower
        # No constructor/connect: this fixture invokes the actual upstream send_action.
        device=PantheraFollower.__new__(PantheraFollower)
        device.teaching_mode=False
        motors=[Mock() for _ in range(7)]
        robot=SimpleNamespace(Motors=motors,get_Gravity=lambda:np.arange(6)*.1,motor_send_cmd=Mock())
        names=[f'motor_{i}' for i in range(7)]
        device.bus=SimpleNamespace(robot=robot,htr_motors=list(motors),target_positions=np.zeros(7),motor_names=names)
        device.motor_map_inv={**{f'joint{i+1}':names[i] for i in range(6)},'gripper':names[6]}
        original=PantheraFollower.send_action
        with tempfile.TemporaryDirectory() as temp:
            trace=install_gripper_trace(Path(temp)/'trace.csv',PantheraFollower)
            trace.assist_config=vars(AssistConfig())
            undo=install_gripper_assist(AssistConfig(),PantheraFollower)
            try:
                steps=[(1.5,1.5,0.)]+[(1.5-.03*i,1.5-.03*i,.1*i) for i in range(1,25)]
                steps += [(.78,.78,2.6),(.95,.78,2.7)]
                for counter,(target,measured,stamp) in enumerate(steps,1):
                    sample=self.state(measured,counter)
                    motors[6].get_current_motor_state.return_value=sample
                    action={**{f'joint{i+1}.pos':.1*(i+1) for i in range(6)},'gripper.pos':target}
                    # Drive the real wrapper with deterministic monotonic time only.
                    from unittest.mock import patch
                    with patch('integrations.panthera_lerobot.gripper_assist.time.monotonic',return_value=stamp):
                        self.assertIs(device.send_action(action),action)
                    self.assertIs(robot.Motors[6],motors[6])
                self.assertEqual(robot.motor_send_cmd.call_count,len(steps))
                for i in range(6):
                    self.assertEqual(motors[i].pos_vel_tqe_kp_kd.call_count,len(steps))
                    args=motors[i].pos_vel_tqe_kp_kd.call_args.args
                    self.assertAlmostEqual(args[0],.1*(i+1))
                    self.assertAlmostEqual(args[2],i*.1)
                    self.assertEqual(args[3:],([11.,23.,23.,18.,15.,1.2][i],
                                               [1.1,2.2,2.2,1.,.9,.12][i]))
                    motors[i].pos_vel_MAXtqe.assert_not_called()
                grip_calls=motors[6].pos_vel_tqe_kp_kd.call_args_list
                self.assertEqual(grip_calls[0].args[2],0.)
                self.assertLess(grip_calls[-2].args[2],0.)
                self.assertEqual(grip_calls[-1].args[2],0.)
                self.assertEqual(grip_calls[-2].args[3:],(4.5,.45))
                motors[6].pos_vel_MAXtqe.assert_not_called()
            finally:
                undo();trace.close()
            self.assertIs(PantheraFollower.send_action,original)

    def test_proxy_is_restored_if_upstream_swallows_motor_error(self):
        motor=Mock();motor.pos_vel_tqe_kp_kd.side_effect=RuntimeError('motor failure')
        motor.get_current_motor_state.return_value=self.state(1.5,1)
        class Follower:
            teaching_mode=False
            def send_action(self,action):
                try:self.bus.robot.Motors[6].pos_vel_tqe_kp_kd(action['gripper.pos'],0.,0.,4.5,.45)
                except RuntimeError:pass
                return action
        device=Follower();motors=[Mock() for _ in range(6)]+[motor]
        device.bus=SimpleNamespace(htr_motors=list(motors),robot=SimpleNamespace(Motors=motors))
        undo=install_gripper_assist(AssistConfig(),Follower)
        try:
            with self.assertRaisesRegex(RuntimeError,'motor failure'):device.send_action({'gripper.pos':1.5})
            self.assertIs(motors[6],motor)
        finally:undo()

    def test_idle_hold_captures_actual_gripper_instead_of_padding_zero(self):
        import threading
        from lerobot_robot_panthera.panthera_follower import PantheraFollower
        device=PantheraFollower.__new__(PantheraFollower)
        device._stop_event=threading.Event();device._last_action_time=0.
        device.fc=[0.]*6;device.fv=[0.]*6;device.vel_threshold=.02
        motors=[Mock() for _ in range(7)]
        robot=SimpleNamespace(Motors=motors,get_current_pos=lambda:np.array([.1]*6),
            get_current_vel=lambda:np.zeros(6),get_current_pos_gripper=lambda:1.6,
            get_Gravity=lambda:np.zeros(6),get_friction_compensation=lambda **kw:np.zeros(6),
            motor_send_cmd=device._stop_event.set)
        device.bus=SimpleNamespace(robot=robot)
        device._control_loop()
        self.assertEqual(motors[6].pos_vel_tqe_kp_kd.call_args.args[0],1.6)
        for motor in motors[:6]:self.assertEqual(motor.pos_vel_tqe_kp_kd.call_args.args[0],.1)

if __name__=='__main__':unittest.main()
