from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            namespace='spot',
            executable='joy_node',
            name='spot_joy',
            parameters=[
                {"deadzone": 0.05}
            ]
        ),
        Node(
            package='commanding',
            namespace='spot',
            executable='joy2cmd',
            name='spot_joy2cmd'
        ),
        Node(
            package='motion_control',
            namespace='spot',
            executable='classic_gait',
            name='spot_gait'
        ),
        Node(
            package='joint_ctl',
            namespace='spot',
            executable='servo_ctl',
            name='spot_servo_ctl'
        )
    ])