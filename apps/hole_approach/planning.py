"""Cancellable two-stage geometry. Pure math, no motor construction."""
import copy
import math
from types import SimpleNamespace
import numpy as np
from scipy.spatial.transform import Rotation
from armhub.task_runtime import OfflineKinematics
from .config import load_config
from armhub.tools import tool_transform,ToolKinematics
from armhub.trajectory import motion_config,retime_feed,ContinuousFeed,JointTransition
from .geometry import target_from_observation, plane_compensation


def solve_pose(robot, q, position, rotation, duration, eps=1e-4):
    robot.q = np.asarray(q).copy()
    seed = np.clip(q, robot.joint_limits['lower'], robot.joint_limits['upper'])
    joints = robot.inverse_kinematics(position.tolist(), rotation, seed, multi_init=False, eps=eps)
    if joints is None:
        joints = robot.inverse_kinematics(position.tolist(), rotation, seed, multi_init=True, eps=eps)
    if joints is None:
        raise ValueError('SDK IK found no solution')
    joints = np.asarray(joints)
    fk = robot.forward_kinematics(joints)
    error = float(np.linalg.norm(np.asarray(fk['position'])-position))
    angle = float(Rotation.from_matrix(rotation.T @ np.asarray(fk['rotation'])).magnitude())
    if error > .002 or angle > math.radians(1):
        raise ValueError('IK residual exceeds 2mm/1deg')
    limits = np.asarray(robot.config['robot']['velocity_limits'])
    return {'start_joints_rad': np.asarray(q).tolist(), 'target_joints_rad': joints.tolist(),
            'target_position_m': position.tolist(), 'target_rotation': rotation.tolist(),
            'target_rpy_deg': Rotation.from_matrix(rotation).as_euler('xyz', degrees=True).tolist(),
            'duration_s': max(duration, float(np.max(np.abs(joints-q)/limits))),
            'ik_position_error_m': error, 'ik_orientation_error_deg': math.degrees(angle)}


def solve_two_stage(request):
    a = SimpleNamespace(**request['options'])
    q = np.asarray(request['q_start']); capture_q = np.asarray(request['q_capture'])
    robot = OfflineKinematics(q, load_config(a.config))
    fk = robot.forward_kinematics(capture_q)
    calibration = np.asarray(request['calibration'])
    tool=tool_transform(a)
    # Camera calibration stays relative to gripper; target pose is the rod tip.
    tool_fk=ToolKinematics(robot,tool).forward_kinematics(capture_q)
    hole, normal, position, rotation = target_from_observation(
        np.asarray(request['center']), np.asarray(request['normal']), fk, calibration, a.standoff, 'align' if a.orientation=='align' else 'keep')
    if a.orientation=='keep':rotation=np.asarray(tool_fk['rotation'])
    offset = plane_compensation(normal, np.asarray(fk['transform']) @ calibration, a.plane_offset_m)
    position += offset
    robot=ToolKinematics(robot,tool)
    first = solve_pose(robot, q, position, rotation, a.duration,eps=1e-6)
    q = np.asarray(first['target_joints_rad']); feed = []; path=[q.tolist()]
    # Dense IK knots define one continuous path; they are never arrival stops.
    count = max(6, int(math.ceil(a.feed_distance/.002)))
    for i in range(1, count+1):
        target = position-normal*(a.feed_distance*i/count)
        segment = solve_pose(robot, q, target, rotation, a.feed_duration/count,eps=1e-6)
        if np.max(np.abs(np.asarray(segment['target_joints_rad'])-q)) > .35:
            raise ValueError('Feed IK changes branch; no motion sent')
        feed.append(segment); q = np.asarray(segment['target_joints_rad']); path.append(q.tolist())
    settings=motion_config(robot.config)
    spec=retime_feed({'duration_s':a.feed_duration,'ramp_fraction':settings['feed_ramp_fraction'],
                     'path_fraction':np.linspace(0,1,count+1).tolist(),'joint_path_rad':path,
                     'start_position_m':position.tolist(),'normal_base':normal.tolist(),'distance_m':a.feed_distance},robot.config)
    trajectory=ContinuousFeed(spec)
    maximum_error=0.
    for fraction in np.linspace(0,1,4*count+1):
        sampled=trajectory.spline(fraction);check=robot.forward_kinematics(sampled)
        error=float(np.linalg.norm(np.asarray(check['position'])-(position-normal*a.feed_distance*fraction)))
        angle=float(Rotation.from_matrix(rotation.T@np.asarray(check['rotation'])).magnitude())
        maximum_error=max(maximum_error,error)
        if error>.002 or angle>math.radians(1):raise ValueError('Interpolated continuous feed deviates from Cartesian line')
    spec['sampled_max_cartesian_error_m']=maximum_error
    math_robot=robot.robot
    approach_curve=JointTransition(np.clip(request['q_start'],math_robot.joint_limits['lower'],math_robot.joint_limits['upper']),
        np.zeros(6),np.zeros(6),first['target_joints_rad'],math_robot.config,first['duration_s'])
    collision_checks={'approach':math_robot.check_trajectory(approach_curve),
                      'feed':math_robot.check_trajectory(trajectory)}
    first['duration_s']=approach_curve.duration
    report = copy.deepcopy(first)
    report.update({'action': 'two_stage_approach', 'source': 'depth_only',
                   'execution_backend':'ArmHub Ruckig + continuous Cartesian feed',
                   'reference_smoothing':'Ruckig 0.12.2 + quintic continuous Cartesian feed', 'motion_sent': False,
                   'frame': 'base_link', 'tcp': 'rod_tip', 'T_gripper_tool':tool.tolist(), 'rod_length_m':getattr(a,'rod_length',.04), 'tcp_xyz_mm':getattr(a,'tcp_xyz_mm',[0,0,0]), 'tcp_rpy_deg':getattr(a,'tcp_rpy_deg',[0,0,0]), 'standoff_m': a.standoff,
                   'feed_distance_m': a.feed_distance, 'final_standoff_m': a.standoff-a.feed_distance,
                   'hole_center_base_m': hole.tolist(), 'outward_normal_base': normal.tolist(),
                   'center_optical_m': request['center'], 'normal_optical': request['normal'],
                   'plane_offset_right_down_m': a.plane_offset_m, 'compensation_base_m': offset.tolist(),
                   'T_tcp_camera': calibration.tolist(), 'calibration_status': request['calibration_status'],
                   'hole_estimate': request.get('hole_estimate'),
                   'collision_checks':collision_checks, 'stage1': first, 'stage2_waypoints': feed,'feed_trajectory':spec,'stage2_point_role':'geometry knots; not arrival stops'})
    return report
