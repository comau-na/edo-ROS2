#!/usr/bin/env python3

import rclpy

from rclpy.node import Node

from vision_msgs.msg import ObjectHypothesis

from vision_msgs.msg import Classification2D

Classification2D

class classification_subscriber(Node):
    
    def __init__(self):
        super().__init__('detection')
        self.subscription = self.create_subscription(
            ObjectHypothesis,
            'classification',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        result = ObjectHypothesis
        self.get_logger().info(result)

        # for i in msg.results:
        #     print(i)
        #     # self.get_logger().info('ID: "%s"' % i)


