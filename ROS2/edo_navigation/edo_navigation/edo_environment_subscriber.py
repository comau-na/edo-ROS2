#!/usr/bin/env python3

import rclpy

import time 

import numpy

from std_msgs.msg import Float32

from rclpy.node import Node

from geometry_msgs.msg import Pose

from std_msgs.msg import Bool

from edo_navigation.edo_scan import image_classifier, image_converter, base_detection





msg = None
recievedImage = False
angle = 0


class navigation_publisher(Node):
    def __init__(self):
        super().__init__("edo_navigation")
        self.publisher_ = self.create_publisher(Pose, "/edo/edo_move",10)


    def publish_coordinates(self,p_x,p_y,p_z,o_x,o_y,o_z,o_w):
        msg = Pose() 
        msg._position.x = p_x
        msg._position.y = p_y
        msg._position.z =  p_z
        msg._orientation.x = o_x
        msg._orientation.y = o_y
        msg._orientation.z = o_z
        msg._orientation.w = o_w
        self.publisher_.publish(msg)
        

 
        
class gripper_publisher(Node):
    def __init__(self):
        super().__init__('gripper')
        self.publisher = self.create_publisher(Float32,'/set_gripper_span',10)
 
    
    def open_close(self, input):
        msg = Float32()
        msg.data = input
        self.publisher.publish(msg)





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
    detection_array = []

    rclpy.init(args=args)
    global msg
    ic = image_converter()
    base = base_detection()

    while rclpy.ok() and ic.recievedImage is False:
        rclpy.spin_once(ic)

    msg = ic.msg 
    
    #getbase here takes in raw image
    base.getBase(msg)
    

    #save detections within detection array
    classifier = image_classifier()
    classifier.classify_objects(msg)
    for obj in classifier.detections:
        detection_array.append(obj)    
    print("detection array contains: ", detection_array)

    for obj in detection_array:
        print("center Coordinates: ", obj.coordinateCenter)
        obj.coordinateCenter = base.get_world_coordinates(obj.coordinateCenter[0],obj.coordinateCenter[1])
        print("world space coordinates: " , obj.coordinateCenter)


    for obj in detection_array:
        if isCube(obj) == True:
            print(obj.classification)
            new_bucket = Bucket(center_x = obj.coordinateCenter[0], 
                                center_y = obj.coordinateCenter[1],
                                classification= obj.classification)
            Bucket_list.append(new_bucket)
            print("cube", isCube(obj))
        if isCube(obj) == False:
            print(obj.classification)
            print("bucket",isCube(obj))
            new_Cube = Cube(center_x = obj.coordinateCenter[0],
                            center_y = obj.coordinateCenter[1],
                            classification= obj.classification,
                            angle= obj.angle
                            )
            Cube_list.append(new_Cube)

    #convert to worldspace
    
            
    #take in empty cubelist,bucketlist,detection array
    print("cubes: ", Cube_list)
    print("buckets: ", Bucket_list)
    
    print("start read environment") 
    #take in cubelist, bucketlist 
    print("start executeCommand")

    for obj in Cube_list:
        print("class: ", obj.classification)
        print("X: ", obj.center_x)
        print("Y: ", obj.center_y)


    
    navpublisher = navigation_publisher()
    grip = gripper_publisher()
        
    for cube in Cube_list:
        for bucket in Bucket_list:
            if colorCheck(cube,bucket) == True:
                #start navigation
                #initialize coordinates
                    p_x = float(0)
                    p_y = float(0)
                    p_z = float(0)
                    o_x = float(0)
                    o_y = float(0)
                    o_z = float(0)
                    o_w = float(0) 
                
                    #starting position
            
                    p_x = -0.413271964035
                    p_y = 0.000747108786946
                    p_z = 1.20756253284
                    o_x = -3.139559138676179
                    o_y = -0.015483082006265326
                    o_z = -0.7787520820355616
                    o_w = float(62.00000)
                    navpublisher.publish_coordinates(p_x,p_y,p_z,o_x,o_y,o_z,o_w)
                    time.sleep(10)
                    print("Starting Position")

                    #cube hover 
                    p_x = cube.center_x
                    p_y = cube.center_y
                    p_z = 1.10756253284
                    o_x = -3.139559138676179
                    o_y = -0.015483082006265326
                    o_z = numpy.pi/4 - numpy.pi/2 + cube.angle
                    o_w = float(62)
                    navpublisher.publish_coordinates(p_x,p_y,p_z,o_x,o_y,o_z,o_w)
                    time.sleep(10)
                    print("Cube Position")


                    #   #gripper open
                
                    input = 0.10
                    grip.open_close(input)
                    time.sleep(10)
                    print("Open Gripper")


                     #cube position (grab)
                    p_x = cube.center_x
                    p_y = cube.center_y
                    p_z = 1.04000
                    o_x = -3.139559138676179
                    o_y = -0.015483082006265326
                    o_z = numpy.pi/4 - numpy.pi/2 + cube.angle
                    o_w = float(62)
                    navpublisher.publish_coordinates(p_x,p_y,p_z,o_x,o_y,o_z,o_w)
                    time.sleep(10)
                    print("Cube Position Grab")

                      #gripper close
                    input = 0.035
                    grip.open_close(input)
                    time.sleep(10)
                    print("Close Gripper")

                    # #cube position (post-grab)
                    p_x = -0.413271964035
                    p_y = 0.000747108786946
                    p_z = 1.20756253284
                    o_x = -3.139559138676179
                    o_y = -0.015483082006265326
                    o_z = numpy.pi/4 - numpy.pi/2 + cube.angle
                    o_w = float(62)
                    navpublisher.publish_coordinates(p_x,p_y,p_z,o_x,o_y,o_z,o_w)
                    time.sleep(10)
                    print("Grab Cube")

                    # #bucket position hover
                    p_x = bucket.center_x
                    p_y = bucket.center_y
                    p_z = 1.180000
                    o_x = -3.139559138676179
                    o_y = -0.015483082006265326
                    o_z = numpy.pi/4 - numpy.pi/2 + cube.angle
                    o_w = float(62)
                    navpublisher.publish_coordinates(p_x,p_y,p_z,o_x,o_y,o_z,o_w)
                    time.sleep(10)
                    print("Hover over bucket")

                    #gripper open
                    input = 0.10
                    grip.open_close(input)
                    time.sleep(10)
                    print("Drop Cube")



    rclpy.shutdown()

       
    
    
    

if __name__ == '__main__':
    main()