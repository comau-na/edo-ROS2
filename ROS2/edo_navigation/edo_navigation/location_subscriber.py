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
        self.get_logger().info('x: "%s"' % msg.x,
                               'y: "%s"' % msg.y,
        )

