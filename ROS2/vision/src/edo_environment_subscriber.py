
import rclpy

import re

from rclpy.node import Node

from vision_msgs.msg._detection2_d_array import Detection2DArray

from vision_msgs.msg._detection2_d import Detection2D

from geometry_msgs.msg import Pose2D



class edo_obj_detection(Node):

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

class obj_location(Node):
    
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

class Block:
    def __init__(self,center_x,center_y, classification):
        self.center_x = center_x
        self.center_y = center_y
        self.classification = classification

    def isBlock(self,classification):
        result = self.classification
        block = "block"
        if result == block:
            return True

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

    def isBucket(self):
        result = self.classification
        bucket = "bucket"
        if result == bucket:
            return True

    def setBucketValues(self,x,y):
        Pose2D.x = self.center_x
        Pose2D.y = self.center_y
    
    def setClassification(self,id):
        Detection2D.id = self.classification
        

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
        if isBucket() == True:
            new_bucket = Bucket()
            new_bucket.setBucketValues(Pose2D.x, Pose2D.y)
            new_bucket.setClassification(Detection2D.id)
            bucket_list.append(new_bucket)


#take in all buckets, blocks from readEnvironment lists
#recieve block and bucket positions
#loop through block list, guide robot to block location
#pick up block
#guide robot to corresponding bucket
#repeat until no blocks

def executeCommanmd(block_list, bucket_list):

    #loop through block list
    count = 0
    for i in block_list:
        count = count +1
        #recieve the block coordinates
        print('Retrieving Coordinates...')
        print('Block number: ' , count)
        curr_block_x = 0.0
        curr_block_y = 0.0
        curr_block_class = "none"
        curr_block = Block()
        block_list[i] = curr_block
        curr_block.center_x = curr_block_x
        curr_block.center_y = curr_block_y
        curr_block.classification = curr_block_class
        print('Block current x position: ', curr_block_x)
        print('Block current y posiiton: ', curr_block_y)
        #loop through bucket list
        for i in bucket_list:
            curr_bucket = Bucket()
            curr_bucket = bucket_list[i]
            #match block with bucket classification
            for word in curr_block_class:
                check_class = re.match(string=curr_bucket.classification)
                if (check_class == True):
                    #receive corresponding bucket coordinates, set as destination
                    destination_x = 0.0
                    destination_y = 0.0
                    curr_bucket.center_x = destination_x
                    curr_bucket.center_y = destination_y
                    print('Bucket destination x: ', destination_x)
                    print('Bucket destination y: ', destination_y)
                    print('Commence sorting operation')

        #start navigation from current block coordinates to destination coordinates


        
    
    




        
def main(args=None):
    rclpy.init(args=args)

    edo_obj_detection = Edo_Obj_Detection()
    obj_location = Obj_Location()

    rclpy.spin(edo_obj_detection)
    rclpy.spin(obj_location)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    edo_obj_detection.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()