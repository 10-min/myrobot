#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class OdomEcho(Node):
    def __init__(self):
        super().__init__('odom_echo')
        self.subscription = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.odom_callback,
            10
        )

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        self.get_logger().info(f'Position -> x: {x:.3f}, y: {y:.3f}, z: {z:.3f}')

def main(args=None):
    rclpy.init(args=args)
    node = OdomEcho()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
