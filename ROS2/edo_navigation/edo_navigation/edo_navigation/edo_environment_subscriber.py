#!/usr/bin/env python3

import rclpy

import numpy

from rclpy.node import Node

from geometry_msgs.msg import Pose

from std_msgs.msg import Bool

from edo_navigation.edo_scan import image_classifier,image_converter

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
        self.publisher = self.create_publisher(Bool,'gripper',10)
 
    
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

    def setAngle(self,angle):
        self.angle = angle
        

    def setCubeValues(self,coordinateCenter):
        self.coordinateCenter = coordinateCenter
    
    def setClassification(self,classification):
        self.classification = classification

class Bucket:
    def __init__(self,center_x,center_y, classification):
        self.center_x = center_x
        self.center_y = center_y
        self.classification = classification

    def setBucketValues(self,coordinateCenter):
        self.coordinateCenter = coordinateCenter
    
    def setClassification(self,classification):
        self.classification = classification


def colorCheck(Cube,Bucket) :
    cube_color_classification = Cube.classification
    bucket_color_classification  = Bucket.classification

    cube_color_classification = cube_color_classification.split('_')[1]
    bucket_color_classification = bucket_color_classification.split('_')[1]

    return(bucket_color_classification == cube_color_classification)
    
    
def isCube(self,classification):
    result = self.classification.split('_')[0]
    return(result == "cube")

def isBucket(self):
    result = self.classification.split('_')[0]
    return(result == "bucket")

Bucket_list = []
Cube_list = []

def readEnvironment():

    
    classifier =  image_classifier()
    detection_array = classifier.detections
    print(detection_array)
    for obj in detection_array:
        print(isCube(obj))
        if isCube(obj) == True:
            print('Cube detected')
            new_Cube = Cube()
            new_Cube.setCubeValues(obj.coordinateCenter) 
            new_Cube.setClassification(obj.classification)
            new_Cube.setAngle(obj.angle)
            Cube_list.append(new_Cube)
            
        if isBucket(obj) == True:
            print('bucket detected')
            new_bucket = Bucket()
            new_bucket.setBucketValues(obj.coordinateCenter)
            new_bucket.setClassification(obj.classification)
            Bucket_list.append(new_bucket)
            print(Cube_list,Bucket_list)
            return(Bucket_list,Cube_list)


# take in all buckets, Cubes from readEnvironment lists
# recieve Cube and bucket positions
# loop through Cube list, guide robot to Cube location
# pick up Cube
# guide robot to corresponding bucket
# repeat until no Cubes



def executeCommand(Cube_list, Bucket_list):   
    #loop through Cube list 
    pub = navigation_publisher()
    grip = gripper_publisher()

    for cube in Cube_list:
        for bucket in Bucket_list:
            if(colorCheck(cube,bucket) == True):
                print(colorCheck(cube,bucket))
                print("starting navigation")
                #start navigation
                
                p_x = 0
                p_y = 0
                p_z = 0
                o_x = 0
                o_y = 0
                o_z = 0
                o_w = 0 
                #initialize coordinates
                #starting position
                p_x = -0.413271964035
                p_y = 0.000747108786946
                p_z = 1.20756253284
                o_x = -3.139559138676179
                o_y = -0.015483082006265326
                o_z = -0.7787520820355616
                o_w = 62.00000
                pub.publish_coordinates(p_x,p_y,p_z,o_x,o_y,o_z,o_w)

                #cube position (hover)
                p_x = cube.coordinateCenter[1]
                p_y = cube.coordinateCenter[2]
                p_z = 1.10756253284
                o_x = -3.139559138676179
                o_y = -0.015483082006265326
                o_z = numpy.pi/4 - numpy.pi/2 + angle
                o_w = 62
                pub.publish_coordinates(p_x,p_y,p_z,o_x,o_y,o_z,o_w)

                #gripper open
                input = 0.35
                gripper_publisher(input)
                #cube position (grab)
                
                p_x = cube.coordinateCenter[1]
                p_y = cube.coordinateCenter[2]
                p_z = 1.04000
                o_x = -3.139559138676179
                o_y = -0.015483082006265326
                o_z = numpy.pi/4 - numpy.pi/2 + angle
                o_w = 62
                pub.publish_coordinates(p_x,p_y,p_z,o_x,o_y,o_z,o_w)


                #gripper close
                input = 0.10
                grip.open_close(input)

                # #cube position (post-grab)
                p_x = -0.413271964035
                p_y = 0.000747108786946
                p_z = 1.20756253284
                o_x = -3.139559138676179
                o_y = -0.015483082006265326
                o_z = numpy.pi/4 - numpy.pi/2 + angle
                o_w = 62
                pub.publish_coordinates(p_x,p_y,p_z,o_x,o_y,o_z,o_w)

                # #bucket position hover
                p_x = bucket.coordinateCenter[1]
                p_y = bucket.coordinateCenter[2]
                p_z = 1.180000
                o_x = -3.139559138676179
                o_y = -0.015483082006265326
                o_z = numpy.pi/4 - numpy.pi/2 + angle
                o_w = 62
                pub.publish_coordinates(p_x,p_y,p_z,o_x,o_y,o_z,o_w)

                 #gripper open
                input = 0.35
                grip.open_close(input)


        # start navigation from current Cube coordinates to destination coordinates
        

    
      
def main(args=None):
    rclpy.init(args=args)
    global msg
    ic = image_converter()

    while rclpy.ok() and ic.recievedImage is False:
        rclpy.spin_once(ic)
    msg = ic.msg 
    
    #getbase here takes in raw image
    

    classifier = image_classifier()
    classifier.classify_objects(msg)

    readEnvironment()
    
     
    executeCommand(Cube_list,Bucket_list)    

    
    # rclpy.destroy_node(gripper_publisher)
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()
