
import rclpy

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

    
def executeCommanmd():
    




        
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