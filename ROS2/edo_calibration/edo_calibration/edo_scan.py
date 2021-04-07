import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from vision_msgs.msg import Classification2D, ObjectHypothesis
from classification_service.srv import GetClassification

import sys
import cv2
import time
import numpy as np
from scipy.spatial import distance as dist
import imutils
from imutils import perspective

recievedImage = False
msg = None

robot_width = 0.270
robot_center_from_edge = 0.230

class Detection:
    def __init__(self, classification, coordinateCenter, confidence, box):
        self.classification = classification
        self.coordinateCenter = coordinateCenter
        self.confidence = confidence
        self.box = box


class image_converter(Node):
    def __init__(self):
        super().__init__('edo_image_converter')

        # subscribe to the raw camera feed
        self.image_subscriber = self.create_subscription(Image, 'edo/camera/image_raw', self.callback, 10)
        self.image_subscriber

        self.bridge = CvBridge()

    def callback(self, img):
        global msg, recievedImage
        print('got an image')
        try:
            msg = self.bridge.imgmsg_to_cv2(img, "bgr8")
        except CvBridgeError as e:
            print(e)
        recievedImage = True


class image_classifier(Node):
    def __init__(self):
        super().__init__('edo_classifier')

        self.classification_client = self.create_client(GetClassification, 'classify_image')
        while not self.classification_client.wait_for_service(timeout_sec=1.5):
            self.get_logger().info('Image Classification service is currently not up...')
        self.req = GetClassification.Request()
        self.bridge = CvBridge()

        self.detections = []      

    def classify_objects(self, imgSrc):
        kernal = np.ones((4, 4), np.uint8)

        img = imgSrc.copy()
       
        topQuarterY = int(imgSrc.shape[0] * .20)
        img = img[:][topQuarterY:, :]
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        img = cv2.GaussianBlur(img, (9, 9), 0)
        img = cv2.Canny(img, 40, 100)
        img = cv2.dilate(img, kernal, iterations=2)
        img = cv2.erode(img, kernal, iterations=1)
        # cv2.imshow("source", img)
        # cv2.waitKey(0)

        contours, hierarchy = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for c in contours:
            # if cv2.contourArea(c) < 10:
            #     continue
            # print("area: ", cv2.contourArea(c) * pixelsToWolrd)
            # compute the rotated bounding box of the contour
            box = cv2.minAreaRect(c)
            box = cv2.boxPoints(box)
            box = np.array(box, dtype="int")
            # order the points in the contour such that they appear
            # in top-left, top-right, bottom-right, and bottom-left
            # order, then draw the outline of the rotated bounding
            # box
            box = perspective.order_points(box)
            box = np.array(box, dtype="int")

            # print("box", box)
            side1, side2 = dist.euclidean(box[0], box[1]), dist.euclidean(box[1], box[2])

            for pnt in box:
                pnt[1] += topQuarterY
            
            # Check if the lengths of the sides are within +-25% of each other 
            if  side2 * 0.75 < side1 < side2 * 1.25:
                xmin, ymin = box[0] 
                xmax, ymax = box[2]
                # Scale the boundaries by 2% on each side
                xminC, yminC, xmaxC, ymaxC = int(xmin * 0.94), int(ymin * 0.94), int(xmax * 1.04), int(ymax * 1.04)
                #print(xmin, xmax, ymin, ymax)
                croppedImg = imgSrc[:][yminC:ymaxC, xminC:xmaxC]
                try:
                #publishing using cv bridge
                    # cv2.imshow("xd", croppedImg)
                    # cv2.waitKey(0)
                    # call service here
                    self.req.img = self.bridge.cv2_to_imgmsg(croppedImg, "bgr8")
                    future = self.classification_client.call_async(self.req)
                    # Spin!!!!! weeeeee
                    rclpy.spin_until_future_complete(self, future)
                    try:
                        response = future.result()
                    except Exception as e:
                        self.get_logger().info('Service call failed %r' % (e,e.what()))

                    # print(response.classification)
                    detection = Detection(response.classification.results[0].id, ((xmin + xmax) // 2,(ymin + ymax) //2),
                                         response.classification.results[0].score, box)
                    self.detections.append(detection)
                except CvBridgeError as e:
                    print(e)


def main(args=None):
    global msg
    rclpy.init(args=args)
    ic = image_converter()

    while rclpy.ok() and recievedImage is False:
        rclpy.spin_once(ic)
    
    classifier = image_classifier()
    classifier.classify_objects(msg)
    print(classifier.detections)
    #rclpy.spin(classifier)

    print("exiting while loop")
    for label in classifier.detections:
        cv2.putText(msg, label.classification,
                    (label.coordinateCenter[0]+20, label.coordinateCenter[1]+40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1,  # font scale
                    (255, 0, 255), 2)  # line type
        cv2.drawContours(msg, [label.box], -1, (0, 255, 0), 2)


    cv2.imshow("testwindow", msg)
    cv2.waitKey(0) 
    #cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
