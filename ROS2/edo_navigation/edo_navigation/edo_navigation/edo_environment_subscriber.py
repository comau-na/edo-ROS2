#!/usr/bin/env python3

import rclpy

from rclpy.node import Node

from vision_msgs.msg import Detection2DArray

from vision_msgs.msg import ObjectHypothesis

from geometry_msgs.msg import Pose2D

from geometry_msgs.msg import Pose

from baseDetection import detection, getObjects





class navigation_publisher(Node):
    def __init__(self):
        super().__init__('edoMove')
        self.publisher_ = self.create_publisher(Pose, 'edoMove',10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self,p_x,p_y,p_z,o_x,o_y,o_z,o_w):
        msg = Pose() 
        msg._position.x = p_x
        msg._position.y = p_y
        msg._position.z = p_z
        msg._orientation.x = o_x
        msg._orientation.y = o_y
        msg._orientation.z = o_z
        msg._orientation.w = o_w
        self.publisher_.publish(msg)

class Cube:
    def __init__(self,center_x,center_y, classification):
        self.center_x = center_x
        self.center_y = center_y
        self.classification = classification

    def setCubeValues(self,coordianteCenter):
        self.coordinateCenter = coordianteCenter
    
    def setClassification(self,classification):
        self.classification = classification

class Bucket:
    def __init__(self,center_x,center_y, classification):
        self.center_x = center_x
        self.center_y = center_y
        self.classification = classification

    def setCubeValues(self,coordianteCenter):
        self.coordinateCenter = coordianteCenter
    
    def setClassification(self,classification):
        self.classification = classification


def colorcheck(Cube,Bucket) :
    cube_color_classification = Cube.classification
    bucket_color_classification  = Bucket.classification

    cube_color_classification = cube_color_classification.split('_')[1]
    bucket_color_classification = bucket_color_classification.split('_')[1]

    return(bucket_color_classification == cube_color_classification)
    
    
def isCube(self,classification):
    result = self.classification
    if result == "cube":
        return True

def isBucket(self):
    result = self.classification
    if result == "bucket":
        return True


def readEnvironment():

    bucket_list = []
    Cube_list = []
    detection_array = getObjects()

    for obj in detection_array:
        if isCube() == True:
            new_Cube = Cube()
            new_Cube.setCubeValues(obj.coordinateCenter) 
            new_Cube.setClassification(obj.classification)
            Cube_list.append(new_Cube)
            print('Cube detected')
        if isBucket() == True:
            new_bucket = Bucket()
            new_bucket.setBucketValues(obj.coordinateCenter)
            new_bucket.setClassification(obj.classification)
            bucket_list.append(new_bucket)
            print('bucket detected')


# take in all buckets, Cubes from readEnvironment lists
# recieve Cube and bucket positions
# loop through Cube list, guide robot to Cube location
# pick up Cube
# guide robot to corresponding bucket
# repeat until no Cubes

def executeCommand(Cube_list, bucket_list):
    
    #loop through Cube list
    count = 0
    for i in Cube_list:
        count = count +1
        #recieve the Cube coordinates
        print('Retrieving Coordinates...')
        print('Cube number: ' , count)
        curr_Cube_x = 0.0
        curr_Cube_y = 0.0
        curr_Cube_class = "none"
        curr_Cube = Cube()
        Cube_list[i] = curr_Cube
        curr_Cube.center_x = curr_Cube_x
        curr_Cube.center_y = curr_Cube_y
        curr_Cube.classification = curr_Cube_class
        print('Cube current x position: ', curr_Cube_x)
        print('Cube current y posiiton: ', curr_Cube_y)
        #loop through bucket list
        for i in bucket_list:
            curr_bucket = Bucket()
            curr_bucket = bucket_list[i]
            curr_bucket.classification = curr_bucket_class
            #match Cube with bucket classification
            result = twoStrings(curr_Cube_class,curr_Cube_class)
            if (result == True):
                #receive corresponding bucket coordinates, set as destination
                destination_x = 0.0
                destination_y = 0.0
                curr_bucket.center_x = destination_x
                curr_bucket.center_y = destination_y
                print('Bucket destination x: ', destination_x)
                print('Bucket destination y: ', destination_y)
                print('Commence sorting operation')

        #start navigation from current Cube coordinates to destination coordinates
        

        
    

      
def main(args=None):
    rclpy.init(args=args)

    #Obj_subscriber = obj_subscriber()
    #location =  loc_subscriber()
    classification_sub = classification_subscriber()
    #rclpy.spin(Obj_subscriber)
    #rclpy.spin(location)
    rclpy.spin(classification_sub)
    readEnvironment()
    #Obj_subscriber.destroy_node()
    #location.destroy_node()
    classification_sub.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()
