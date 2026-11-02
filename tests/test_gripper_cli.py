import unittest
from unittest.mock import patch
import test_panthera_hardware as hardware_fixture
from motion_fixture import virtual_time
from examples.gripper import move_gripper


class GripperCliTests(unittest.TestCase):
    make_arm=hardware_fixture.HardwareTests.make_arm

    def test_open_reaches_target_and_keeps_six_axes(self):
        virtual_time(self)
        arm,driver=self.make_arm();arm.enable()
        result=move_gripper(arm,1.6,.3,.3)
        self.assertEqual(result['status'],'target_reached')
        self.assertEqual([driver.transmissions[0][i][0] for i in range(6)],[.1]*6)
        self.assertFalse(result['grasp_confirmed'])

    def test_blocked_close_holds_without_claiming_grasp(self):
        clock=virtual_time(self)
        arm,driver=self.make_arm();arm.enable()
        original=driver.motor_send_cmd
        def blocked_gripper():
            before=driver.states[6].position
            original()
            driver.states[6].position=before
        driver.motor_send_cmd=blocked_gripper
        with patch('examples.gripper.time',clock):
            result=move_gripper(arm,0.,.3,.3,allow_contact=True)
        self.assertEqual(result['status'],'stopped_before_target')
        self.assertFalse(result['grasp_confirmed'])
        self.assertFalse(arm._moving)
        self.assertEqual(driver.transmissions[-1][6][0],.5)

if __name__=='__main__':unittest.main()
