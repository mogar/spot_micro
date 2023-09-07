#!/usr/bin/env python3

# Servo Controller
# Listen to servo commands and set servo to 
# commanded position.

import rclpy
from rclpy.node import Node

def main(args=None):
    rclpy.init(args=args)
    node = Node("servo_ctl")
    node.get_logger().info("Hello servo-ctl")
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
