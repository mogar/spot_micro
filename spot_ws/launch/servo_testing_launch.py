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
                # Setting Up Joystick:
                # - Get Number (you will see something like jsX): ls /dev/input/
                # - Make available to ROS: sudo chmod a+rw /dev/input/jsX
                # - Make sure the dev param is correct below
                # You can ignore this msg: [ERROR] [1591631380.406690714]: Couldn't open joystick force feedback!
                # It just means your controller is missing some functionality, but this package doesn't use it. 
                {"dev": "/dev/input/js0"},
                {"deadzone": 0.05}
            ]
        ),
        Node(
            package='commanding',
            namespace='spot',
            executable='joy2joints',
            name='spot_joy2joints'
        ),
        Node(
            package='joint_ctl',
            namespace='spot',
            executable='servo_ctl',
            name='spot_servo_ctl'
        )
    ])