#!/usr/bin/env python3

import rclpy

from rclpy.node import Node

from geometry_msgs.msg import Pose2D





class loc_subscriber(Node):
    
    def __init__(self):
        super().__init__('detection')
        self.subscription = self.create_subscription(
            Pose2D,
            '/detection',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        msg = Pose2D()
        self.get_logger().info('x: "%s"' % msg.x,
                               'y: "%s"' % msg.y,
        )


def main(args=None):
    rclpy.init(args=args)

    location =  loc_subscriber()


    rclpy.spin(location)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    location.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
