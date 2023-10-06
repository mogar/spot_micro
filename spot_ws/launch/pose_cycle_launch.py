from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='motion_control',
            namespace='spot',
            executable='pose_cycle',
            name='pose_cycle'
        ),
        Node(
            package='joint_ctl',
            namespace='spot',
            executable='servo_ctl',
            name='spot_servo_ctl'
        )
    ])