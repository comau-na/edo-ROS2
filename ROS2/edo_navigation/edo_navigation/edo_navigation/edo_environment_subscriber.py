#!/usr/bin/env python3

import rclpy

import time 

import numpy

from rclpy.node import Node

from geometry_msgs.msg import Pose

from std_msgs.msg import Bool

from edo_navigation.edo_scan import image_classifier, image_converter





msg = None
recievedImage = False
angle = 0

class Detection:
    def __init__(self, classification, coordinateCenter, confidence, box, angle):
        self.classification = classification
        self.coordinateCenter = coordinateCenter
        self.confidence = confidence
        self.box = box
        self.angle = angle

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
        self.publisher = self.create_publisher(Bool,'open_gripper',10)
 
    
    def open_close(self, input):
        msg = Bool()
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



def executeCommand(Cube_list, Bucket_list):   
    #loop through Cube list 
    pub = navigation_publisher()
    grip = gripper_publisher()
    
    for cube in Cube_list:
        for bucket in Bucket_list:
            if colorCheck(cube,bucket) == True:
                print("starting navigation")
                #start navigation
                #initialize coordinates
                p_x = 0
                p_y = 0
                p_z = 0
                o_x = 0
                o_y = 0
                o_z = 0
                o_w = float(0) 
                
                #starting position
                
                p_x = -0.413271964035
                p_y = 0.000747108786946
                p_z = 1.20756253284
                o_x = -3.139559138676179
                o_y = -0.015483082006265326
                o_z = -0.7787520820355616
                o_w = float(62.00000)
                pub.publish_coordinates(p_x,p_y,p_z,o_x,o_y,o_z,o_w)
                

                #cube position (hover)
                
                p_x = cube.center_x
                p_y = cube.center_y
                p_z = 1.10756253284
                o_x = -3.139559138676179
                o_y = -0.015483082006265326
                o_z = numpy.pi/4 - numpy.pi/2 + cube.angle
                o_w = float(62)
                pub.publish_coordinates(p_x,p_y,p_z,o_x,o_y,o_z,o_w)
        

                #gripper open
                
                input = True
                grip.open_close(input)
                #cube position (grab)
                
                
                p_x = cube.center_x
                p_y = cube.center_y
                p_z = 1.04000
                o_x = -3.139559138676179
                o_y = -0.015483082006265326
                o_z = numpy.pi/4 - numpy.pi/2 + cube.angle
                o_w = float(62)
                pub.publish_coordinates(p_x,p_y,p_z,o_x,o_y,o_z,o_w)


                #gripper close
                input = False
                grip.open_close(input)

                # #cube position (post-grab)
                p_x = -0.413271964035
                p_y = 0.000747108786946
                p_z = 1.20756253284
                o_x = -3.139559138676179
                o_y = -0.015483082006265326
                o_z = numpy.pi/4 - numpy.pi/2 + cube.angle
                o_w = float(62)
                pub.publish_coordinates(p_x,p_y,p_z,o_x,o_y,o_z,o_w)

                # #bucket position hover
                p_x = bucket.center_x
                p_y = bucket.center_y
                p_z = 1.180000
                o_x = -3.139559138676179
                o_y = -0.015483082006265326
                o_z = numpy.pi/4 - numpy.pi/2 + cube.angle
                o_w = float(62)
                pub.publish_coordinates(p_x,p_y,p_z,o_x,o_y,o_z,o_w)

                 #gripper open
                input = True
                grip.open_close(input)

                rclpy.shutdown()
        # start navigation from current Cube coordinates to destination coordinates
        

    
      
def main(args=None):

    
    Bucket_list = []
    Cube_list = []
    detection_array = []

    rclpy.init(args=args)
    global msg
    ic = image_converter()

    while rclpy.ok() and ic.recievedImage is False:
        rclpy.spin_once(ic)

    msg = ic.msg 
    
    #getbase here takes in raw image
    
    #save detections within detection array
    classifier = image_classifier()
    classifier.classify_objects(msg)
    for obj in classifier.detections:
        detection_array.append(obj)    
    print("detection array contains: ", detection_array)
    for obj in detection_array:
        if isCube(obj) == True:
            print(obj.classification)
            new_bucket = Bucket(center_x = float(obj.coordinateCenter[0]), 
                                center_y = float(obj.coordinateCenter[1]),
                                classification= obj.classification)
            Bucket_list.append(new_bucket)
            print("cube", isCube(obj))
        if isCube(obj) == False:
            print(obj.classification)
            print("bucket",isCube(obj))
            new_Cube = Cube(center_x = float(obj.coordinateCenter[0]),
                            center_y = float(obj.coordinateCenter[1]),
                            classification= obj.classification,
                            angle= obj.angle
                            )
            Cube_list.append(new_Cube)
            
    #take in empty cubelist,bucketlist,detection array
    print("cubes: ", Cube_list)
    print("buckets: ", Bucket_list)
    
    print("start read environment")
    # readEnvironment(detection_array, Bucket_list, Cube_list)    
    #take in cubelist, bucketlist 
    print("start executeCommand")
    while rclpy.ok():
        navpublisher = navigation_publisher()
        
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
                    print("first move finished")

                    #cube hover 
                    p_x = float(cube.center_x)
                    p_y = float(cube.center_y)
                    p_z = 1.10756253284
                    o_x = -3.139559138676179
                    o_y = -0.015483082006265326
                    o_z = numpy.pi/4 - numpy.pi/2 + cube.angle
                    o_w = float(62)
                    navpublisher.publish_coordinates(p_x,p_y,p_z,o_x,o_y,o_z,o_w)
                    time.sleep(10)
                    print("second move finished")







    rclpy.shutdown()

       
    
    
    

if __name__ == '__main__':
    main()