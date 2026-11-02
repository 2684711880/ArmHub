#!/usr/bin/env python3
"""在当前已启动机型上验证共享空间命令、不可达拒绝及碰撞拒绝。仅仿真。"""
import argparse
from dataclasses import asdict
import json
import math
from pathlib import Path

from armhub import create_arm


def main():
    from geometry_msgs.msg import Pose
    from moveit_msgs.msg import CollisionObject
    from moveit_msgs.srv import ApplyPlanningScene
    from shape_msgs.msg import SolidPrimitive

    parser = argparse.ArgumentParser()
    parser.add_argument('--robot', required=True, choices=['panthera_ht', 'nero7'])
    args = parser.parse_args()
    folder = Path('validation') / args.robot
    folder.mkdir(parents=True, exist_ok=True)
    report = {'robot': args.robot, 'backend': 'simulation', 'checks': []}
    with create_arm(args.robot, 'simulation') as arm:
        arm.enable()
        report['initial_pose'] = arm.get_pose()
        for xyz in [(0.25, 0.0, 0.30), (0.22, 0.08, 0.32)]:
            result = arm.move_to(xyz)
            report['checks'].append({'name': 'move_to', 'passed': True, 'result': asdict(result)})

        def expect_rejected(name, xyz):
            before = arm.get_pose()['position_m']
            try:
                arm.move_to(xyz)
            except RuntimeError as error:
                delta = math.dist(before, arm.get_pose()['position_m'])
                if delta > 0.005:
                    raise AssertionError(f'{name}: 拒绝后末端移动 {delta} m')
                report['checks'].append({'name': name, 'passed': True,
                                         'reason': str(error), 'position_drift_m': delta})
            else:
                raise AssertionError(f'{name}: 目标意外执行成功')

        expect_rejected('unreachable_target_rejected', (5.0, 5.0, 5.0))
        service = arm.node.create_client(ApplyPlanningScene, '/apply_planning_scene')
        if not service.wait_for_service(timeout_sec=5):
            raise RuntimeError('规划场景服务未就绪')

        def obstacle(operation):
            request = ApplyPlanningScene.Request()
            request.scene.is_diff = True
            obj = CollisionObject(id='armhub_validation_blocker', operation=operation)
            obj.header.frame_id = 'base_link'
            if operation == CollisionObject.ADD:
                primitive = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.5, 0.5, 0.5])
                pose = Pose()
                pose.position.x, pose.position.y, pose.position.z = arm.get_pose()['position_m']
                pose.orientation.w = 1.0
                obj.primitives = [primitive]
                obj.primitive_poses = [pose]
            request.scene.world.collision_objects = [obj]
            if not arm._wait(service.call_async(request), 5).success:
                raise RuntimeError('更新障碍物失败')

        obstacle(CollisionObject.ADD)
        try:
            expect_rejected('colliding_start_rejected', (0.25, 0.0, 0.30))
        finally:
            obstacle(CollisionObject.REMOVE)
        report['checks'].append({'name': 'recovery_after_obstacle_removed', 'passed': True,
                                 'result': asdict(arm.move_to((0.25, 0.0, 0.30)))})
        report['final_pose'] = arm.get_pose()
    report['passed'] = all(check['passed'] for check in report['checks'])
    path = folder / 'acceptance.json'
    path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
