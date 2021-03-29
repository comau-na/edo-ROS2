#!/usr/bin/env python3

import rclpy

from rclpy.node import Node

from vision_msgs.msg import ObjectHypothesis








class classification_subscriber(Node):
    
    def __init__(self):
        super().__init__('detection')
        self.subscription = self.create_subscription(
            ObjectHypothesis,
            '/detection',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('ID: "%s"' % msg.id)


