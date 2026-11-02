#!/usr/bin/env python3
"""生成两种独立单臂模型和统一控制配置；保留官方几何、惯量及关节限制。"""
import copy
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import xacro
import yaml

ROOT = Path(__file__).resolve().parents[1]


def add(parent, tag, **attributes):
    return ET.SubElement(parent, tag, {key: str(value) for key, value in attributes.items()})


def write_xml(path, root):
    ET.indent(root)
    ET.ElementTree(root).write(path, encoding='unicode', xml_declaration=False)


def prepare(robot):
    profile_path = ROOT / f'armhub/robots/{robot}.json'
    profile = json.loads(profile_path.read_text())
    target = ROOT / f'robots/{robot}'
    for name in ['description', 'simulation', 'planning']:
        (target / name).mkdir(parents=True, exist_ok=True)
    if robot == 'panthera_ht':
        source = ROOT / 'vendor/panthera/description/urdf/panthera_ht_ros_description_gripper.xacro'
        model = ET.fromstring(xacro.process_file(str(source)).toxml())
        for mesh in model.iter('mesh'):
            mesh.set('filename', mesh.get('filename').replace(
                'package://panthera_ht_description/', 'package://armhub_sim/vendor/panthera/description/'))
        semantic_file = ROOT / 'vendor/panthera/moveit_reference/panthera_ht_ros_description_gripper.srdf'
        gripper = ['L_finger_joint']
        initial = {**{f'joint{i}': 0.0 for i in range(1, 7)}, 'L_finger_joint': 0.02, 'R_finger_joint': -0.02}
        tip = 'gripper_center'
    else:
        model = ET.parse(ROOT / 'vendor/agilex/nero/robot_with_camera.urdf').getroot()
        # 去除相机及其固定子树；本阶段只接机械臂与夹爪。
        removed = set()
        for joint in model.findall('joint'):
            if joint.get('name') == 'camera_stand_joint':
                removed.add(joint.find('child').get('link'))
        changed = True
        while changed:
            changed = False
            for joint in model.findall('joint'):
                if joint.find('parent').get('link') in removed:
                    child = joint.find('child').get('link')
                    if child not in removed:
                        removed.add(child)
                        changed = True
        for element in list(model):
            if ((element.tag == 'link' and element.get('name') in removed)
                    or (element.tag == 'joint' and element.find('child').get('link') in removed)):
                model.remove(element)
        for mesh in model.iter('mesh'):
            mesh.set('filename', mesh.get('filename').replace(
                'package://agx_arm_description/agx_arm_urdf/nero/', 'package://armhub_sim/vendor/agilex/nero/'))
        semantic_file = ROOT / 'vendor/agilex/moveit_reference/nero.srdf'
        gripper = ['gripper_joint1', 'gripper_joint2']
        initial = {**{f'joint{i}': 0.0 for i in range(1, 8)}, 'gripper_joint1': 0.02, 'gripper_joint2': -0.02}
        tip = 'gripper_base'
        profile['joint_names'] = [f'joint{i}' for i in range(1, 8)]
        profile['name'] = 'NERO 七轴单臂（官方模型）'

    model.set('name', f'armhub_{robot}')
    # 统一安装场景。基座在世界 z=0.75 m；所有 API 空间目标仍相对 base_link。
    for element in list(model):
        if element.tag in ['gazebo', 'ros2_control', 'mujoco']:
            model.remove(element)
        elif element.tag == 'joint' and element.find('parent').get('link') == 'world':
            model.remove(element)
    if model.find("link[@name='world']") is None:
        add(model, 'link', name='world')
    mount = add(model, 'joint', name='armhub_mount', type='fixed')
    add(mount, 'parent', link='world')
    add(mount, 'child', link='base_link')
    add(mount, 'origin', xyz='0 0 0.75')
    floor = add(model, 'link', name='armhub_floor')
    for kind in ['visual', 'collision']:
        shape = add(floor, kind)
        add(shape, 'origin', xyz='0 0 -0.025')
        add(add(shape, 'geometry'), 'box', size='8 8 0.05')
    joint = add(model, 'joint', name='armhub_floor_fixed', type='fixed')
    add(joint, 'parent', link='world')
    add(joint, 'child', link='armhub_floor')
    # 物理与规划共用同一份网格碰撞模型；已有运动关节不随意改惯量/重力。
    write_xml(target / 'description/planning.urdf', model)

    control = add(model, 'ros2_control', name='ArmHubGazeboSystem', type='system')
    add(add(control, 'hardware'), 'plugin').text = 'gazebo_ros2_control/GazeboSystem'
    for name, value in initial.items():
        entry = add(control, 'joint', name=name)
        if name == 'R_finger_joint':
            add(entry, 'param', name='mimic').text = 'L_finger_joint'
            add(entry, 'param', name='multiplier').text = '-1.0'
        else:
            add(entry, 'command_interface', name='position')
        state = add(entry, 'state_interface', name='position')
        add(state, 'param', name='initial_value').text = str(value)
        add(entry, 'state_interface', name='velocity')
    plugin = add(add(model, 'gazebo'), 'plugin', name='gazebo_ros2_control', filename='libgazebo_ros2_control.so')
    add(plugin, 'parameters').text = '__CONTROLLERS_FILE__'
    write_xml(target / 'description/simulation.urdf', model)

    original_semantic = ET.parse(semantic_file).getroot()
    semantic = ET.Element('robot', name=model.get('name'))
    group = add(semantic, 'group', name='arm')
    add(group, 'chain', base_link='base_link', tip_link=tip)
    group = add(semantic, 'group', name='gripper')
    for name in gripper:
        add(group, 'joint', name=name)
    add(semantic, 'end_effector', name='gripper', parent_link=tip, group='gripper', parent_group='arm')
    home = add(semantic, 'group_state', name='home', group='arm')
    for name in profile['joint_names']:
        add(home, 'joint', name=name, value=initial[name])
    links = {entry.get('name') for entry in model.findall('link')}
    for entry in original_semantic.findall('disable_collisions'):
        if entry.get('link1') in links and entry.get('link2') in links:
            semantic.append(copy.deepcopy(entry))
    write_xml(target / 'planning/robot.srdf', semantic)

    controllers = {'controller_manager': {'ros__parameters': {
        'update_rate': 100, 'use_sim_time': True,
        'joint_state_broadcaster': {'type': 'joint_state_broadcaster/JointStateBroadcaster'},
        'arm_controller': {'type': 'joint_trajectory_controller/JointTrajectoryController'},
        'gripper_controller': {'type': 'joint_trajectory_controller/JointTrajectoryController'},
    }}}
    controllers['joint_state_broadcaster'] = {'ros__parameters': {
        'joints': profile['joint_names'] + gripper, 'interfaces': ['position', 'velocity']}}
    moveit_controllers = {'controller_names': ['arm_controller', 'gripper_controller']}
    for controller_name, names in [('arm_controller', profile['joint_names']), ('gripper_controller', gripper)]:
        controllers[controller_name] = {'ros__parameters': {
            'joints': names, 'command_interfaces': ['position'], 'state_interfaces': ['position', 'velocity'],
            'allow_partial_joints_goal': False, 'state_publish_rate': 50.0, 'action_monitor_rate': 20.0,
            'constraints': {'goal_time': 3.0, 'stopped_velocity_tolerance': 0.02}}}
        moveit_controllers[controller_name] = {'type': 'FollowJointTrajectory',
            'action_ns': 'follow_joint_trajectory', 'default': True, 'joints': names}
    (target / 'simulation/controllers.yaml').write_text(yaml.safe_dump(controllers, sort_keys=False))
    (target / 'planning/moveit_controllers.yaml').write_text(yaml.safe_dump({
        'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
        'moveit_simple_controller_manager': moveit_controllers}, sort_keys=False))
    (target / 'planning/kinematics.yaml').write_text(yaml.safe_dump({'arm': {
        'kinematics_solver': 'kdl_kinematics_plugin/KDLKinematicsPlugin',
        'kinematics_solver_search_resolution': 0.005, 'kinematics_solver_timeout': 0.1}}))
    limits = {}
    for name in profile['joint_names'] + gripper:
        limit = model.find(f"joint[@name='{name}']/limit")
        limits[name] = {'has_velocity_limits': True, 'max_velocity': float(limit.get('velocity')),
                       'has_acceleration_limits': True, 'max_acceleration': 1.0 if name.startswith('joint') else 0.04}
    (target / 'planning/joint_limits.yaml').write_text(yaml.safe_dump({
        'default_velocity_scaling_factor': 0.15, 'default_acceleration_scaling_factor': 0.15,
        'joint_limits': limits}))
    runtime = {'robot': robot, 'planning_group': 'arm', 'base_frame': 'base_link', 'tcp_frame': tip,
               'joint_names': profile['joint_names'], 'joint_states_topic': '/joint_states',
               'trajectory_action': '/arm_controller/follow_joint_trajectory',
               'initial_positions': initial, 'model_name': model.get('name')}
    (target / 'robot.json').write_text(json.dumps(runtime, ensure_ascii=False, indent=2) + '\n')
    profile['status']['simulation'] = 'implemented_see_validation'
    profile['status']['model'] = f'独立模型与 MoveIt 配置：robots/{robot}/'
    profile['references']['simulation_controller'] = runtime['trajectory_action']
    profile_path.write_text(json.dumps(profile, ensure_ascii=False, indent=2) + '\n')
    print(f'Prepared {robot}: {len(profile["joint_names"])} joints, TCP={tip}')


if __name__ == '__main__':
    for robot in ['panthera_ht', 'nero7']:
        prepare(robot)
