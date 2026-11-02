#!/usr/bin/env python3
"""RealSense color/CameraInfo + real joint-state FK. No motor interface."""
import argparse
import json
import math
import time
from array import array as ByteArray
from pathlib import Path

import numpy as np
import pyrealsense2 as rs
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, JointState
from tf2_ros import TransformBroadcaster

from .kinematics import quaternion
from geometry_msgs.msg import TransformStamped
from .kinematics import ArmFK
from .realsense_rectify import build_maps, rectify


class Inputs(Node):
    def __init__(self, joint_source='ros', serial=None):
        super().__init__('handeye_real_inputs')
        self.fk = ArmFK()
        self.tf = TransformBroadcaster(self)
        self.frames = self.joints = self.markers = 0
        self.last_joint = None
        self.last_image = None
        self.image = self.create_publisher(Image, '/handeye/color/image_raw', 10)
        self.info = self.create_publisher(CameraInfo, '/handeye/color/camera_info', 10)
        self.joint_source=joint_source
        self.gravity_log_dir=Path(__file__).resolve().parents[2]/'logs/panthera_ht'
        self.last_log_stamp=None
        if joint_source=='ros':
            self.create_subscription(JointState, '/joint_states', self.on_joint, 10)
        else:
            self.create_timer(.03,self.on_gravity_log)
        self.pipe = rs.pipeline()
        cfg = rs.config()
        if serial:cfg.enable_device(serial)
        cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        profile=self.pipe.start(cfg)
        try:
            intr=profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
            self.rectification=build_maps(intr)
        except BaseException:
            self.pipe.stop()
            raise
        self.create_timer(1 / 30, self.on_frame)
        self.get_logger().info(f'Rectified color/CameraInfo publishing; feedback source: {joint_source}. No motor commands.')

    def on_gravity_log(self):
        directory=self.gravity_log_dir
        files=sorted(directory.glob('*.jsonl'),key=lambda p:p.stat().st_mtime,reverse=True)
        for path in files[:3]:
            if time.time()-path.stat().st_mtime>1.:continue
            with path.open('rb') as f:
                f.seek(max(0,path.stat().st_size-65536))
                lines=f.read().splitlines()
            ticks=[];ended=False
            for line in reversed(lines):
                try:r=json.loads(line)
                except (ValueError,UnicodeError):continue
                if r.get('event') in ('gravity_result','gravity_exit_hold_confirmed','gravity_failure','transport_closing'):ended=True;break
                if r.get('event')=='gravity_tick':ticks.append(r);break
            if ended or not ticks:continue
            r=ticks[0];age=time.monotonic()-r['monotonic_s']
            if not 0<=age<=.15 or r['monotonic_s']==self.last_log_stamp:continue
            q=r.get('position_rad',[])
            if len(q)!=6 or not np.isfinite(q).all():continue
            msg=JointState();msg.name=[f'joint{i}' for i in range(1,7)];msg.position=list(q)
            stamp=time.time()-age;msg.header.stamp.sec=int(stamp);msg.header.stamp.nanosec=int((stamp-int(stamp))*1e9)
            self.on_joint(msg);self.last_log_stamp=r['monotonic_s'];return

    def on_joint(self, msg):
        stamp = msg.header.stamp
        age = time.time() - (stamp.sec + stamp.nanosec * 1e-9)
        positions = dict(zip(msg.name, msg.position))
        names = [f'joint{i}' for i in range(1, 7)]
        if not -.02 <= age <= .15 or not all(n in positions and math.isfinite(positions[n]) for n in names):
            return
        t = self.fk.transform(positions, 'base_link', 'gripper_center')
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = 'base_link'
        tf.child_frame_id = 'gripper_center'
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = map(float, t[:3, 3])
        q = quaternion(t[:3, :3])
        tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w = map(float, q)
        self.tf.sendTransform(tf)
        self.joints += 1
        self.last_joint = time.monotonic()

    def on_frame(self):
        frames = self.pipe.poll_for_frames()
        if not frames:
            return
        color = frames.get_color_frame()
        if not color:
            return
        # Timestamp is host receive time. Initial sampling is stationary only.
        stamp = self.get_clock().now().to_msg()
        intr = color.profile.as_video_stream_profile().get_intrinsics()
        array = rectify(np.asanyarray(color.get_data()),self.rectification)
        image = Image()
        image.header.stamp = stamp
        image.header.frame_id = 'handeye_color_optical'
        image.height, image.width = array.shape[:2]
        image.encoding = 'bgr8'
        image.is_bigendian = False
        image.step = image.width * 3
        # A typed byte array bypasses ROS Python's per-element validation of
        # a 921600-byte sequence, which otherwise starves the joint callback.
        image.data = ByteArray('B', array.tobytes())
        info = CameraInfo()
        info.header = image.header
        info.width, info.height = intr.width, intr.height
        info.distortion_model = 'plumb_bob'
        info.d = [0.] * 5
        info.k = [intr.fx, 0., intr.ppx, 0., intr.fy, intr.ppy, 0., 0., 1.]
        info.r = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
        info.p = [intr.fx, 0., intr.ppx, 0., 0., intr.fy, intr.ppy, 0., 0., 0., 1., 0.]
        self.info.publish(info)
        self.image.publish(image)
        self.frames += 1
        self.last_image = time.monotonic()
        if self.frames % 15 == 0:
            import cv2
            _, ids, _ = cv2.aruco.detectMarkers(array, cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250))
            if ids is not None and 23 in ids:
                self.markers += 1

    def summary(self):
        now = time.monotonic()
        return {'images': self.frames, 'accepted_joint_messages': self.joints,
                'marker23_checks_passed': self.markers,
                'joint_age_s': None if self.last_joint is None else now - self.last_joint,
                'image_age_s': None if self.last_image is None else now - self.last_image,
                'no_motor_commands': True, 'timestamp_policy': 'host receive; stationary sampling required'}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--check-seconds', type=float, default=0.)
    parser.add_argument('--report')
    parser.add_argument('--joint-source',choices=['ros','gravity-log'],default='ros')
    parser.add_argument('--serial')
    args = parser.parse_args()
    rclpy.init()
    node = None
    try:
        node = Inputs(args.joint_source,args.serial)
        deadline = time.monotonic() + args.check_seconds
        while rclpy.ok() and (args.check_seconds <= 0 or time.monotonic() < deadline):
            rclpy.spin_once(node, timeout_sec=.1)
        result = node.summary()
        print(json.dumps(result, indent=2))
        if args.report:
            report=Path(args.report)
            report.parent.mkdir(parents=True,exist_ok=True)
            report.write_text(json.dumps(result, indent=2))
    except KeyboardInterrupt:
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        if node is not None:
            node.pipe.stop()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
