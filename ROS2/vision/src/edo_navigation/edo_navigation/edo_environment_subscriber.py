#!/usr/bin/env python3

import rclpy

from rclpy.node import Node

from vision_msgs.msg import Detection2DArray

from vision_msgs.msg import Detection2D

from geometry_msgs.msg import Pose2D

from geometry_msgs.msg import Pose



class obj_subscriber(Node):

    def __init__(self):
        super().__init__('detection')
        self.subscription = self.create_subscription(
            Detection2DArray,
            'detection',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

class loc_subscriber(Node):
    
    def __init__(self):
        super().__init__('detection')
        self.subscription = self.create_subscription(
            Pose2D,
            'detection',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

class navigation_publisher(Node):
    def __init__(self):
        super().__init__('edoMove')
        self.publisher_ = self.create_publisher(Pose, 'edoMove',10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Pose() 
        msg._orientation.x = 'Pose x: %d ' % self.i
        msg._orientation.y = 'Pose y: %d ' % self.i
        msg._orientation.z  = 'Pose z: %d ' % self.i 
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

class Block:
    def __init__(self,center_x,center_y, classification):
        self.center_x = center_x
        self.center_y = center_y
        self.classification = classification

    def setBlockValues(self,x,y):
        Pose2D.x = self.center_x
        Pose2D.y = self.center_y
    
    def setClassification(self,id):
        Detection2D.id = self.classification

class Bucket:
    def __init__(self,center_x,center_y, classification):
        self.center_x = center_x
        self.center_y = center_y
        self.classification = classification

    def setBucketValues(self,x,y):
        Pose2D.x = self.center_x
        Pose2D.y = self.center_y
    
    def setClassification(self,id):
        Detection2D.id = self.classification
        
def listToString(list1):
    str1 =""
    for char in list1:
        str1 += char
        return str1
   
def twoStrings(s1, s2) :
    col1=[]
    col2 =[]
    for char in s1[0:3]:
        col1.append(char)
    for char in s2[0:3]:
        col2.append(char)
    print(col1,col2)
    classcolor1 = listToString(col1)
    classcolor2 = listToString(col2)
    if classcolor1 == classcolor2:
        return True

def isBlock(self,classification):
        result = self.classification
        block = "block"
        if result == block:
            return True

def isBucket(self):
    result = self.classification
    bucket = "bucket"
    if result == bucket:
        return True


def readEnvironment():
    object_array = []
    bucket_list = []
    block_list = []
    Detection2DArray.detections = object_array
    for i in object_array:
        if isBlock() == True:
            new_block = Block()
            new_block.setBlockValues(Pose2D.x,Pose2D.y)
            new_block.setClassification(Detection2D.id)
            block_list.append(new_block)
            print('block detected')
        if isBucket() == True:
            new_bucket = Bucket()
            new_bucket.setBucketValues(Pose2D.x, Pose2D.y)
            new_bucket.setClassification(Detection2D.id)
            bucket_list.append(new_bucket)
            print('bucket detected')


#take in all buckets, blocks from readEnvironment lists
#recieve block and bucket positions
#loop through block list, guide robot to block location
#pick up block
#guide robot to corresponding bucket
#repeat until no blocks

# def executeCommand(block_list, bucket_list):
    
#     #loop through block list
#     count = 0
#     for i in block_list:
#         count = count +1
#         #recieve the block coordinates
#         print('Retrieving Coordinates...')
#         print('Block number: ' , count)
#         curr_block_x = 0.0
#         curr_block_y = 0.0
#         curr_block_class = "none"
#         curr_block = Block()
#         block_list[i] = curr_block
#         curr_block.center_x = curr_block_x
#         curr_block.center_y = curr_block_y
#         curr_block.classification = curr_block_class
#         print('Block current x position: ', curr_block_x)
#         print('Block current y posiiton: ', curr_block_y)
#         #loop through bucket list
#         for i in bucket_list:
#             curr_bucket = Bucket()
#             curr_bucket = bucket_list[i]
#             curr_bucket.classification =  curr_bucket_class
#             #match block with bucket classification
#             result = twoStrings(curr_block_class,curr_block_class)
#             if (result == True):
#                 #receive corresponding bucket coordinates, set as destination
#                 destination_x = 0.0
#                 destination_y = 0.0
#                 curr_bucket.center_x = destination_x
#                 curr_bucket.center_y = destination_y
#                 print('Bucket destination x: ', destination_x)
#                 print('Bucket destination y: ', destination_y)
#                 print('Commence sorting operation')

#         #start navigation from current block coordinates to destination coordinates


        
    

        
def main(args=None):
    rclpy.init(args=args)

    Obj_subscriber = obj_subscriber()
    location =  loc_subscriber()
    nav_publisher = navigation_publisher()

    rclpy.spin(Obj_subscriber)
    rclpy.spin(location)
    rclpy.spin(nav_publisher)

    readEnvironment()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    Obj_subscriber.destroy_node()
    location.destroy_node()
    nav_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()