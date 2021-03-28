import rclpy

from rclpy.node import Node

from geometry_msgs.msg import Pose



class move_publisher(Node):
    def __init__(self):
        super().__init__('edoMove')
        self.publisher_ = self.create_publisher(Pose, 'edoMove',10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Pose() 
        msg._orientation.x = 'Pose orientation x: %d ' % self.i
        msg._orientation.y = 'Pose orientation y: %d ' % self.i
        msg._orientation.z  = 'Pose orientation z: %d ' % self.i 
        msg._orientation.w = 'Pose orientation w: %d '% self.i
        msg._position.x = 'Pose position x: %d' % self.i
        msg._position.y = 'Pose position y: %d' % self.i
        msg._position.z = 'Pose position z: %d' % self.i 
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)


    nav_publisher = move_publisher()


    rclpy.spin(nav_publisher)


    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)

    nav_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()