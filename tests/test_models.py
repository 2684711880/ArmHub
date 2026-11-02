import hashlib
import json
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

from armhub import create_arm

ROOT = Path(__file__).resolve().parents[1]


class ModelTests(unittest.TestCase):
    def test_meshes_are_local_and_exist(self):
        for robot in ['panthera_ht', 'nero7']:
            for kind in ['planning', 'simulation']:
                root = ET.parse(ROOT / f'robots/{robot}/description/{kind}.urdf').getroot()
                for mesh in root.iter('mesh'):
                    uri = mesh.get('filename')
                    self.assertTrue(uri.startswith('package://armhub_sim/'), uri)
                    self.assertTrue((ROOT / uri.removeprefix('package://armhub_sim/')).is_file(), uri)

    def test_joint_mapping_and_tcp_exist(self):
        for robot in ['panthera_ht', 'nero7']:
            config = json.loads((ROOT / f'robots/{robot}/robot.json').read_text())
            root = ET.parse(ROOT / f'robots/{robot}/description/simulation.urdf').getroot()
            arm = create_arm(robot, 'simulation')
            self.assertEqual(list(arm.profile.joint_names), config['joint_names'])
            for name in config['joint_names']:
                self.assertIsNotNone(root.find(f"joint[@name='{name}']/limit"))
                self.assertIsNotNone(root.find(f"ros2_control/joint[@name='{name}']"))
            self.assertIsNotNone(root.find(f"link[@name='{config['tcp_frame']}']"))
            self.assertEqual(root.find('ros2_control/hardware/plugin').text, 'gazebo_ros2_control/GazeboSystem')

    def test_model_and_planner_use_same_geometry(self):
        for robot in ['panthera_ht', 'nero7']:
            planning = ET.parse(ROOT / f'robots/{robot}/description/planning.urdf').getroot()
            simulation = ET.parse(ROOT / f'robots/{robot}/description/simulation.urdf').getroot()
            for kind in ['link', 'joint']:
                def canonical(item):
                    return ET.canonicalize(ET.tostring(item, encoding='unicode').strip(), strip_text=True)
                expected = [canonical(item) for item in planning.findall(kind)]
                actual = [canonical(item) for item in simulation.findall(kind)]
                self.assertEqual(expected, actual)

    def test_vendor_snapshot_matches_lock(self):
        lock = json.loads((ROOT / 'deps/models.lock.json').read_text())
        for filename, digest in lock['files'].items():
            self.assertEqual(hashlib.sha256((ROOT / filename).read_bytes()).hexdigest(), digest, filename)


if __name__ == '__main__':
    unittest.main()
