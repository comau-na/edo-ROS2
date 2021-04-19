#!/usr/bin/env python3

import rclpy
import time 
import numpy

from std_msgs.msg import Float32
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool

from edo_navigation.edo_scan import image_classifier, image_converter, base_detection

class navigation_publisher(Node):
    def __init__(self):
        super().__init__("edo_navigation")
        self.publisher = self.create_publisher(Pose, "/edo/edo_move", 10)
        self.subscriber = self.create_subscription(Bool, 'edo/move_success',self.callback, 10)
        self.gripper_publisher = self.create_publisher(Float32,'edo/set_gripper_span',10)

        self.is_executed = False
        self.recievedState = False  
        self.allow_grip = False

    def callback(self, msg):
        self.is_executed = True
        self.executionState = msg.data

    def publish_coordinates(self,p_x,p_y,p_z,o_x,o_y,o_z,o_w, grip_command="none"):
        msg = Pose() 
        msg._position.x = p_x
        msg._position.y = p_y
        msg._position.z =  p_z
        msg._orientation.x = o_x
        msg._orientation.y = o_y
        msg._orientation.z = o_z
        msg._orientation.w = o_w
        self.publisher.publish(msg)

        while rclpy.ok() and self.is_executed is False:
            rclpy.spin_once(self)

        if self.executionState is False:
            state = "failed"
            # Reset to candle position after failure
            msg._position.x = 0.0
            msg._position.y = 0.0
            msg._position.z =  2.1
            msg._orientation.x = 0.0
            msg._orientation.y = 0.0
            msg._orientation.z = 0.0
            msg._orientation.w = 62.0
            self.publisher.publish(msg)
            raise Exception("Failed robot path")
        else:
            state = "success"
            if grip_command != "none":
                grip_span = Float32()
                if grip_command is "close":
                    print("Closing gripper")
                    grip_span.data = 0.035
                elif grip_command is "open":
                    print("Opening gripper")
                    grip_span.data = 0.100
                self.gripper_publisher.publish(grip_span)
                time.sleep(1.5)

        print("Execution state:", state)
        self.is_executed = False

    def set_gripper_span(self, span):
        grip_span = Float32()
        grip_span.data = span
        self.gripper_publisher.publish(grip_span)

                
class Cube:
    def __init__(self,center_x,center_y, classification, angle):
        self.center_x = center_x
        self.center_y = center_y
        self.classification = classification
        self.angle = angle


class Bucket:
    def __init__(self,center_x,center_y, classification):
        self.center_x = center_x
        self.center_y = center_y
        self.classification = classification


def colorCheck(Cube,Bucket) :
    cube_color_classification = Cube.classification
    bucket_color_classification  = Bucket.classification

    cube_color_classification = cube_color_classification.split('_')[1]
    bucket_color_classification = bucket_color_classification.split('_')[1]

    return(bucket_color_classification == cube_color_classification)


def isCube(obj):
    result = obj.classification.split('_')[0]
    return(result == "cube")
      

def main(args=None):
    Bucket_list = []
    Cube_list = []

    rclpy.init(args=args)
    ic = image_converter()
    base = base_detection()
    navpublisher = navigation_publisher()

    while rclpy.ok() and ic.recievedImage is False:
        rclpy.spin_once(ic)

    # Hack to get two images which clears the ghost images
    ic.recievedImage = False
    while rclpy.ok() and ic.recievedImage is False:
        rclpy.spin_once(ic)
    
    #getbase here takes in raw image
    base.getBase(ic.msg)
    
    #save detections within detection array
    classifier = image_classifier()
    classifier.classify_objects(ic.msg)
    detection_array = classifier.detections

    for obj in detection_array:
        print("center Coordinates: ", obj.coordinateCenter)
        obj.coordinateCenter = base.get_world_coordinates(obj.coordinateCenter[0],obj.coordinateCenter[1])
        print("world space coordinates: " , obj.coordinateCenter)

    for obj in detection_array:
        print(obj.classification)
        if isCube(obj) == False:
            new_bucket = Bucket(center_x = obj.coordinateCenter[0], 
                                center_y = obj.coordinateCenter[1],
                                classification= obj.classification)
            Bucket_list.append(new_bucket)
            print("cube", isCube(obj))
        else:
            print("bucket",isCube(obj))
            new_Cube = Cube(center_x = obj.coordinateCenter[0],
                            center_y = obj.coordinateCenter[1],
                            classification= obj.classification,
                            angle= obj.angle)
            Cube_list.append(new_Cube)
    
    print("start read environment") 
    #take in cubelist, bucketlist 
    print("start executeCommand")

    for obj in Cube_list:
        print("class: ", obj.classification)
        print("X: ", obj.center_x)
        print("Y: ", obj.center_y)
    
    o_x = -3.139559138676179
    o_y = -0.015483082006265326

    for cube in Cube_list:
        for bucket in Bucket_list:
            if colorCheck(cube,bucket) == True:
                #start navigation
                #initialize coordinates
                print("moving cube", cube.classification, "into bucket", bucket.classification)
                o_z = numpy.pi/4 - numpy.pi/2 + cube.angle

                temp = cube.center_x 
                cube.center_x = -cube.center_y
                cube.center_y = -temp

                temp = bucket.center_x 
                bucket.center_x = -bucket.center_y
                bucket.center_y = -temp
            
                #cube hover 
                print("Hover over cube position")
                p_x = cube.center_x
                p_y = cube.center_y
                p_z = 1.10756253284
                print("Going to", p_x, p_y)
                status = navpublisher.publish_coordinates(p_x,p_y,p_z,o_x,o_y,o_z, 62.0, "open")
                
                # cube position (grab)
                print("Lowering position")
                p_x = cube.center_x
                p_y = cube.center_y
                p_z = 1.040006
                navpublisher.publish_coordinates(p_x,p_y,p_z,o_x,o_y,o_z, 62.0, "close")
                print("main loop status", status)
      
                # cube position (post-grab)
                print("Rise up")
                p_x = cube.center_x
                p_y = cube.center_y
                p_z = 1.20756253284
                navpublisher.publish_coordinates(p_x,p_y,p_z,o_x,o_y,o_z, 62.0)

                # bucket position hover
                print("Hover over bucket")
                p_x = bucket.center_x
                p_y = bucket.center_y
                p_z = 1.20756253284
                navpublisher.publish_coordinates(p_x,p_y,p_z,o_x,o_y,o_z, 69420.0, "open")

    
    # Return home
    navpublisher.publish_coordinates(0.0, 0.0, 2.1, 0.0, 0.0, 0.0, 62.0)
    rclpy.shutdown()

if __name__ == '__main__':
    main()