#!/usr/bin/env python3

import rclpy

from rclpy.node import Node

from geometry_msgs.msg import Pose

from edo_calibration.edo_scan import image_classifier,image_converter

msg = None
recievedImage = False



class navigation_publisher(Node):
    def __init__(self):
        super().__init__('edoMove')
        self.publisher_ = self.create_publisher(Pose, 'edoMove',10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)


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


def readEnvironment():

    Bucket_list = []
    Cube_list = []
    classifier =  image_classifier()
    detection_array = classifier.detections

    for obj in detection_array:
        if isCube(obj) == True:
            print('Cube detected')
            new_Cube = Cube()
            new_Cube.setCubeValues(obj.coordinateCenter) 
            new_Cube.setClassification(obj.classification)
            Cube_list.append(new_Cube)
            
        if isBucket(obj) == True:
            print('bucket detected')
            new_bucket = Bucket()
            new_bucket.setBucketValues(obj.coordinateCenter)
            new_bucket.setClassification(obj.classification)
            Bucket_list.append(new_bucket)
            
            return(Bucket_list,Cube_list)


# take in all buckets, Cubes from readEnvironment lists
# recieve Cube and bucket positions
# loop through Cube list, guide robot to Cube location
# pick up Cube
# guide robot to corresponding bucket
# repeat until no Cubes

# def executeCommand(Cube_list, Bucket_list):   
#     #loop through Cube list 

#     for cube in Cube_list:
#         print("Number of Cubes: " , len(Cube_list))
#         for bucket in Bucket_list:
#             if(colorCheck(cube,bucket) == True):
#                 #start navigation
           

        #start navigation from current Cube coordinates to destination coordinates
        

        
    

      
def main(args=None):
    rclpy.init(args=args)
    global msg
    ic = image_converter()

    while rclpy.ok() and recievedImage is False:
        rclpy.spin_once(ic)
    
    classifier = image_classifier()
    classifier.classify_objects(msg)
    print(classifier.detections)

    rclpy.spin(navigation_publisher)

    readEnvironment()
    rclpy.destroy_node(navigation_publisher)
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()
