import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class GestureSubscriber(Node):

    #subscriber constructor
    def __init__(self):
        super().__init__('gesture_subscriber')
        self.subscription = self.create_subscription(
            String,
            'gesture_class',
            self.gesture_callback,
            100)
        self.subscription

    #print gesture message to screen
    def gesture_callback(self, msg):
        self.get_logger().info('Detected hand gesture type: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)

    #node creation and spin (for callbacks)
    gesture_subscriber = GestureSubscriber()
    rclpy.spin(gesture_subscriber)

    # destroy node or done by garbage can
    gesture_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
