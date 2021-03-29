#!/usr/bin/env python3

import rclpy


from rclpy.node import Node

from vision_msgs.msg import Detection2DArray




class obj_subscriber(Node):

    def __init__(self):
        super().__init__('detection')
        self.subscription = self.create_subscription(
            Detection2DArray,
            '/detection',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('Number of objects detected: "%s"' % len(msg.detections))


