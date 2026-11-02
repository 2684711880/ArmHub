""" Static transform publisher acquired via MoveIt 2 hand-eye calibration """
""" EYE-IN-HAND: gripper_center -> handeye_color_optical """
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    nodes = [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="log",
            arguments=[
                "--frame-id",
                "gripper_center",
                "--child-frame-id",
                "handeye_color_optical",
                "--x",
                "-0.0849784",
                "--y",
                "0.0132455",
                "--z",
                "0.058796",
                "--qx",
                "-0.575352",
                "--qy",
                "0.57639",
                "--qz",
                "-0.411959",
                "--qw",
                "0.408698",
                # "--roll",
                # "3.12748",
                # "--pitch",
                # "1.23815",
                # "--yaw",
                # "1.58265",
            ],
        ),
    ]
    return LaunchDescription(nodes)
