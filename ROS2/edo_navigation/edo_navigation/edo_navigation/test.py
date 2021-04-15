import rclpy
 
import numpy

import time

from rclpy.node import Node

from geometry_msgs.msg import Pose



class navigation_publisher(Node):
    def __init__(self):
        super().__init__("edoNav")

        self.publisher = self.create_publisher(Pose, "/edo/edo_move", 10)


    def publish_coordinates(self):
        msg = Pose() 
        msg.position.x = -0.413271964035
        msg.position.y = 0.000747108786946
        msg.position.z =  1.20756253284
        msg.orientation.x = -3.139559138676179
        msg.orientation.y = -0.015483082006265326
        msg.orientation.z = -0.7787520820355616
        msg.orientation.w = 62.000
        self.publisher.publish(msg)
        print(msg)


def main(args=None):
    
    rclpy.init(args=args)
  
    print("publishing test")
    navpublisher = navigation_publisher()
    
    print("spin publisher")


    while rclpy.ok():
        #rclpy.spin_once(navpublisher)
        navpublisher.publish_coordinates()
        print("im publishing")
        time.sleep(1)

    rclpy.shutdown()
    print("shutting down node")
    

if __name__ == '__main__':
    main()
