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
        msg = ObjectHypothesis()
        self.get_logger().info('ID: "%s"' % msg.id)





def main(args=None):
    rclpy.init(args=args)


    classification_sub = classification_subscriber()


    rclpy.spin(classification_sub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

