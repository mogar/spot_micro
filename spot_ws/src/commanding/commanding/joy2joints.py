
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

        self.get_logger().info("Joy2Joints initialized")

    def joy_callback(self, msg):
        joint_msg = JointAngles()

        if msg.axes[0] > 0:
            self.get_logger().info(">0")
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
