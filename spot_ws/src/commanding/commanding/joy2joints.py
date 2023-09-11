
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from spot_interfaces.msg import JointAngles

class Joy2Joints(Node):
    def __init__(self):
        super().__init__('joy2joints')
        self._joint_pub = self.create_publisher(JointAngles, 'spot/joints', 10)
        self._joy_sub = self.create_subscription(Joy, "joy", self.joy_callback, 10)
        self._joy_sub # prevent unused variable warning

        # Set up parameters used in parsing joy messages
        self.declare_parameter('frequency', 200.0)

        self.declare_parameter('axis_linear_x', 2)
        self.declare_parameter('axis_linear_y', 3)
        self.declare_parameter('axis_linear_z', 1)

        self.declare_parameter('axis_angular', 0)

        self.declare_parameter('scale_linear', 1.0)
        self.declare_parameter('scale_angular', 1.0)

        self.declare_parameter('button_switch', 0)
        self.declare_parameter('button_estop', 1)

        self.get_logger().info("Joy2Joints initialized")

    def joy_callback(self, msg):
        joint_msg = JointAngles()

        x_linear = msg.axes[self.get_parameter('axis_linear_x').get_parameter_value().int_value]
        if x_linear > 0:
            self.get_logger().info("axis_linear_x: " + str(x_linear))
            joint_msg.fls = 90
        else:
            joint_msg.fls = 0

        self._joint_pub.publish(joint_msg)


def main(args=None):
    print('Hi from joy2joints.')

    rclpy.init(args=args)
    node = Joy2Joints()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
