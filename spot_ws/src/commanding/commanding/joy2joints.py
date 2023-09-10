
import rclpy
from rclpy.node import Node

from spot_interfaces.msg import JointAngles

class Joy2Joints(Node):
    def __init__(self):
        super().__init__('joy2joints')
        # TODO initialize joy listener
        self._publisher = self.create_publisher(JointAngles, 'spot/joints', 10)

        self.get_logger().info("Joy2Joints initialized")




def main(args=None):
    print('Hi from joy2joints.')

    rclpy.init(args=args)
    node = Joy2Joints()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
