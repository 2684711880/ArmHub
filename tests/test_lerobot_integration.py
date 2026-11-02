"""Offline lifecycle checks: no SDK constructors or physical devices."""
import ast
import importlib.util
import os
from pathlib import Path
import subprocess
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

HERE = Path(__file__).resolve().parents[1]/'integrations/panthera_lerobot'
spec = importlib.util.spec_from_file_location('armhub_lerobot_runtime_test', HERE/'armhub_lerobot_runtime.py')
runtime = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runtime)


class EnvironmentTests(unittest.TestCase):
    def test_runner_exposes_matching_viewer_without_shell_activation(self):
        root = HERE.parents[1]
        result = subprocess.run(
            ['bash', str(root/'scripts/with_lerobot.sh'), '-c',
             'import importlib.metadata, pathlib, shutil, subprocess, sys; '
             'viewer=shutil.which("rerun"); assert viewer, "viewer missing from PATH"; '
             'assert pathlib.Path(viewer).parent == pathlib.Path(sys.executable).parent; '
             'version=subprocess.check_output([viewer,"--version"],text=True); '
             'assert "rerun-cli "+importlib.metadata.version("rerun-sdk")+" " in version'],
            env=dict(os.environ, PATH=os.defpath), capture_output=True, text=True, timeout=30)
        self.assertEqual(result.returncode, 0, result.stdout+result.stderr)


class LifecycleTests(unittest.TestCase):
    def device(self, alive=False):
        events=[]
        thread=Mock()
        thread.is_alive.return_value=alive
        thread.join.side_effect=lambda **kw:events.append('joined')
        bus=Mock(is_connected=True)
        bus.disconnect.side_effect=lambda:events.append('closed')
        camera=Mock(is_connected=True)
        camera.disconnect.side_effect=lambda:events.append('camera_closed')
        return SimpleNamespace(_thread=thread,_stop_event=Mock(),bus=bus,cameras={'wrist':camera}),events

    def test_exit_stops_writer_before_hold_and_release(self):
        device,events=self.device()
        with patch.object(runtime,'hold_current',side_effect=lambda bus:events.append('held')):
            runtime.disconnect_device(device)
        self.assertEqual(events,['joined','held','closed','camera_closed'])
        device._stop_event.set.assert_called_once()

    def test_hold_failure_is_not_reported_as_success_and_releases_resources(self):
        device,events=self.device()
        with patch.object(runtime,'hold_current',side_effect=RuntimeError('stale feedback')):
            with self.assertRaisesRegex(RuntimeError,'stale feedback'):
                runtime.disconnect_device(device)
        self.assertEqual(events,['joined','closed','camera_closed'])

    def test_live_writer_prevents_second_sender(self):
        device,events=self.device(alive=True)
        with patch.object(runtime,'hold_current') as hold:
            with self.assertRaisesRegex(RuntimeError,'控制线程未退出'):
                runtime.disconnect_device(device)
        hold.assert_not_called()
        device.bus.disconnect.assert_not_called()

    def test_preset_entrypoints_return_before_upstream_motion(self):
        for name in ('panthera_follower.py','panthera_leader.py'):
            path=HERE/'Panthera-HT_lerobot/Panthera-HT_robot_lerobot/src/lerobot_robot_panthera'/name
            tree=ast.parse(path.read_text())
            methods=[n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name in
                     ('move_to_initial_position','move_to_zero_position')]
            self.assertEqual(len(methods),2)
            for method in methods:
                code=compile(ast.Module(body=[method],type_ignores=[]),str(path),'exec')
                namespace={'logger':Mock()}
                exec(code,namespace)
                # An empty object has no motors: any residual motion call fails.
                namespace[method.name](object())


if __name__=='__main__':
    unittest.main()
