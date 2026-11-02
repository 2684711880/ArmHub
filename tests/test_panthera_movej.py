import unittest
from unittest.mock import Mock
import numpy as np

from armhub.adapters.panthera_movej import SDKMoveJBridge
from armhub.motion import JointTransition
from motion_fixture import virtual_time
from test_panthera_cartesian import FakeKinematics
import test_panthera_hardware as fixture


class SDKMoveJTests(unittest.TestCase):
    make_arm=fixture.HardwareTests.make_arm

    def test_calls_official_movej_and_holds_all_seven_slots(self):
        arm,driver=self.make_arm();arm.enable()
        states=arm.read_feedback();bridge=SDKMoveJBridge(arm,[.1]*6)
        bridge.official_moveJ=Mock(wraps=bridge.official_moveJ)
        target=[.11,.1,.1,.1,.1,.1]
        caps=bridge.send(target,states,.5,.2,.005)
        self.assertEqual(bridge.official_moveJ.call_count,1)
        self.assertFalse(bridge.official_moveJ.call_args.kwargs['iswait'])
        self.assertEqual(len(driver.transmissions),1)
        packet=driver.transmissions[0]
        self.assertEqual(set(packet),set(range(7)))
        self.assertAlmostEqual(packet[0][1],(.11-.1)/.2)
        self.assertEqual(packet[6][0],.5)
        self.assertEqual(caps[1],.005)

    def test_feedback_based_velocity_excess_is_rejected_before_send(self):
        arm,driver=self.make_arm();arm.enable();states=arm.read_feedback()
        bridge=SDKMoveJBridge(arm,[.1]*6)
        with self.assertRaisesRegex(ValueError,'速度超限'):
            bridge.send([.2]*6,states,.5,.05,.005)
        self.assertEqual(driver.transmissions,[])

    def test_cancel_guard_rejects_packet(self):
        arm,driver=self.make_arm();arm.enable();states=arm.read_feedback()
        bridge=SDKMoveJBridge(arm,[.1]*6,lambda:False)
        with self.assertRaisesRegex(RuntimeError,'替代'):
            bridge.send([.101]*6,states,.5,.05,.005)
        self.assertEqual(driver.transmissions,[])

    def test_nonfinite_and_out_of_bounds_targets_never_send(self):
        arm,driver=self.make_arm();arm.enable();states=arm.read_feedback()
        bridge=SDKMoveJBridge(arm,[.1]*6)
        for target in ([np.nan]*6,[10.]*6):
            with self.assertRaises((ValueError,RuntimeError)):
                bridge.send(target,states,.5,.05,.005)
        self.assertEqual(driver.transmissions,[])

    def test_complete_sdk_movej_reference_session_and_stop(self):
        clock=virtual_time(self);arm,driver=self.make_arm();arm._kinematics_cache=FakeKinematics()
        events=[];arm._record=lambda event,**data:events.append(dict(event=event,**data))
        arm.enable()
        curve=JointTransition((.1,)*6,(.12,.1,.1,.1,.1,.1),arm.config,speed_limit=.1,minimum_duration=2.)
        arm.start_reference_trajectory(curve,command_mode='sdk_movej')
        for _ in range(1000):
            session=arm._reference_execution
            clock.sleep(max(0,session.next_due-clock.monotonic()))
            result=arm.advance_reference_trajectory()
            if result is not None:break
        else:self.fail('moveJ reference never completed')
        self.assertFalse(arm._moving)
        refs=[e for e in events if e['event']=='stream_reference']
        self.assertGreater(len(refs),30)
        self.assertTrue(all(e['command_mode']=='sdk_movej' for e in refs))
        self.assertTrue(all(len(e['velocity_caps_rad_s'])==6 for e in refs))
        self.assertTrue(all(packet[6][0]==.5 for packet in driver.transmissions))
        # A second session remains cancellable through the same hold path.
        allowed=[True]
        reverse=JointTransition((.12,.1,.1,.1,.1,.1),(.1,)*6,arm.config,speed_limit=.1,minimum_duration=2.)
        arm.start_reference_trajectory(reverse,command_guard=lambda:allowed[0],command_mode='sdk_movej')
        arm.advance_reference_trajectory();allowed[0]=False
        with self.assertRaisesRegex(RuntimeError,'替代'):arm.advance_reference_trajectory()
        self.assertFalse(arm._moving)


if __name__=='__main__':unittest.main()
