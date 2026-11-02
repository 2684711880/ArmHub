import json
import os
from pathlib import Path
import socket
from urllib.parse import urlparse

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, Shutdown
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def setup(context):
    share = Path(get_package_share_directory('armhub_sim'))
    robot = LaunchConfiguration('robot').perform(context)
    if robot not in ['panthera_ht', 'nero7']:
        raise ValueError('robot 必须是 panthera_ht / nero7')
    uri = urlparse(os.environ.get('GAZEBO_MASTER_URI', 'http://127.0.0.1:11488'))
    try:
        with socket.create_connection((uri.hostname, uri.port), timeout=0.5):
            pass
    except OSError:
        pass
    else:
        raise RuntimeError('当前 ArmHub Gazebo 端口已占用；先在旧启动终端 Ctrl+C，再切换模型')
    folder = share / 'robots' / robot
    def read_yaml(path):
        return yaml.safe_load(path.read_text())
    simulation = (folder / 'description/simulation.urdf').read_text()
    simulation = simulation.replace('package://armhub_sim/', share.as_uri() + '/')
    simulation = simulation.replace('__CONTROLLERS_FILE__', str(folder / 'simulation/controllers.yaml'))
    planning = (folder / 'description/planning.urdf').read_text().replace('package://armhub_sim/', share.as_uri() + '/')
    description = {'robot_description': planning}
    semantic = {'robot_description_semantic': (folder / 'planning/robot.srdf').read_text()}
    kinematics = {'robot_description_kinematics': read_yaml(folder / 'planning/kinematics.yaml')}
    limits = {'robot_description_planning': read_yaml(folder / 'planning/joint_limits.yaml')}
    ompl = {'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': ' '.join([
                'default_planner_request_adapters/AddTimeOptimalParameterization',
                'default_planner_request_adapters/ResolveConstraintFrames',
                'default_planner_request_adapters/FixWorkspaceBounds',
                'default_planner_request_adapters/FixStartStateBounds',
                'default_planner_request_adapters/FixStartStateCollision',
                'default_planner_request_adapters/FixStartStatePathConstraints']),
            'start_state_max_bounds_error': 0.1,
            'planner_configs': {'RRTConnectkConfigDefault': {'type': 'geometric::RRTConnect', 'range': 0.0}},
            'arm': {'planner_configs': ['RRTConnectkConfigDefault']},
            'gripper': {'planner_configs': ['RRTConnectkConfigDefault']}}
    configs = [description, semantic, kinematics, limits,
               {'planning_pipelines': ['ompl'], 'default_planning_pipeline': 'ompl', 'ompl': ompl},
               read_yaml(folder / 'planning/moveit_controllers.yaml'),
               {'use_sim_time': True, 'publish_robot_description': False, 'publish_robot_description_semantic': True,
                'publish_planning_scene': True, 'publish_geometry_updates': True,
                'publish_state_updates': True, 'publish_transforms_updates': True,
                'trajectory_execution.allowed_start_tolerance': 0.01}]
    spawn = Node(package='gazebo_ros', executable='spawn_entity.py',
                 arguments=['-entity', f'armhub_{robot}', '-topic', 'robot_description'], output='screen')
    controllers = [Node(package='controller_manager', executable='spawner',
                        arguments=[name, '--controller-manager-timeout', '90'], output='screen')
                   for name in ['joint_state_broadcaster', 'arm_controller', 'gripper_controller']]
    return [
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            str(Path(get_package_share_directory('gazebo_ros')) / 'launch/gazebo.launch.py')),
            launch_arguments={'world': str(share / 'worlds/bench.world'), 'gui': LaunchConfiguration('gui')}.items()),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': simulation, 'use_sim_time': True}], output='screen'),
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=lambda event, context:
            controllers if event.returncode == 0 else [Shutdown(reason='仿真模型生成失败')])), spawn,
        Node(package='moveit_ros_move_group', executable='move_group', parameters=configs, output='screen'),
        Node(package='rviz2', executable='rviz2', arguments=['-d', str(share / 'config/moveit.rviz')],
             parameters=configs, condition=IfCondition(LaunchConfiguration('rviz')), output='log'),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='panthera_ht'),
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        OpaqueFunction(function=setup),
    ])
