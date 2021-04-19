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
from math import atan

class Detection:
    def __init__(self, classification, coordinateCenter, confidence, box, angle):
        self.classification = classification
        self.coordinateCenter = coordinateCenter
        self.confidence = confidence
        self.box = box
        self.angle = angle


class image_converter(Node):
    def __init__(self):
        super().__init__('edo_image_converter')

        # subscribe to the raw camera feed
        self.image_subscriber = self.create_subscription(Image, 'edo/camera/image_raw', self.callback, 10)
        self.image_subscriber

        self.msg = None
        self.recievedImage = False

        self.bridge = CvBridge()

    def callback(self, img):
        print('got an image')
        try:
            self.msg = self.bridge.imgmsg_to_cv2(img, "bgr8")
        except CvBridgeError as e:
            print(e)
        self.recievedImage = True


class image_classifier(Node):
    def __init__(self):
        super().__init__('edo_classifier')

        self.classification_client = self.create_client(GetClassification, 'classify_image')
        while not self.classification_client.wait_for_service(timeout_sec=1.5):
            self.get_logger().info('Image Classification service is currently not up...')
        self.req = GetClassification.Request()
        self.bridge = CvBridge()
        self.contourImg = None
        self.robot_ycord = 0

        self.detections = []      

    def classify_objects(self, imgSrc):
        kernal = np.ones((4, 4), np.uint8)

        img = imgSrc.copy()
       
        # topQuarterY = int(imgSrc.shape[0] * .20)
        img = img[:][self.robot_ycord:, :]
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        img = cv2.GaussianBlur(img, (9, 9), 0)
        img = cv2.Canny(img, 40, 100)
        img = cv2.dilate(img, kernal, iterations=2)
        img = cv2.erode(img, kernal, iterations=1)
        # cv2.imshow("source", img)
        # cv2.waitKey(0)

        contours, hierarchy = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for c in contours:
            if cv2.contourArea(c) < 200:
                continue
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
                pnt[1] += self.robot_ycord
            
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
                        self.get_logger().info('Service call failed %r' % e)

                    # Find the rotation of the cube in radians 
                    deltaX = box[0][1] - box[1][1]
                    deltaY = box[1][0] - box[0][0]
                    angle = atan(deltaY / deltaX)
                    print(angle)

                    # print(response.classification)
                    detection = Detection(response.classification.results[0].id, ((xmin + xmax) // 2,(ymin + ymax) //2),
                                         response.classification.results[0].score, box, angle)
                    print(detection.classification, detection.angle)

                    cv2.putText(self.contourImg, detection.classification,
                        (detection.coordinateCenter[0]+20, detection.coordinateCenter[1]+40),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1,  # font scale
                        (255, 0, 255), 2)  # line type
                    cv2.drawContours(self.contourImg, [detection.box], -1, (0, 255, 0), 2)

                    self.detections.append(detection)

                except CvBridgeError as e:
                    print(e)

        cv2.imshow("base_detection", self.contourImg)
        cv2.waitKey(0)
    
class base_detection(Node):
    def __init__(self):
        super().__init__('edo_base_detection')

        self.robot_ycord = 0
        self.robot_midpoint = 0
        self.pixelsToWolrd = 0.0

        self.declare_parameter('sim', True)
        self.is_simulation = self.get_parameter('sim')
        self.contourImg = None

        self.ROBOT_WIDTH = 0.270

        if self.is_simulation:
            self.ROBOT_CENTER_FROM_EDGE = 0.230
        else:
            self.ROBOT_CENTER_FROM_EDGE = 0.070

    def getBase(self, imgSrc):
        kernal = np.ones((4, 4), np.uint8)

        # Important! img.shape is y(height), x(width) for some reason  
        topQuarterY = int(imgSrc.shape[0] * .25)
        xCordStart = int(imgSrc.shape[1]* 0.25)
        xCordEnd = int(imgSrc.shape[1]*0.75)
        croppedImg = imgSrc[:][:topQuarterY, xCordStart:xCordEnd]

        grayImg = cv2.cvtColor(croppedImg, cv2.COLOR_BGR2GRAY)
        blurImg = cv2.GaussianBlur(grayImg, (9, 9), 0)
        cannyImg = cv2.Canny(blurImg, 100, 150)
        dialatedImg = cv2.dilate(cannyImg, kernal, iterations=2)
        croppedImg = cv2.erode(dialatedImg, kernal, iterations=1)

        #print("cropped image shape", croppedImg.shape, xCordStart, xCordEnd)
        contours, hierarchy = cv2.findContours(croppedImg, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Get the biggest contour which should be the edo base
        contours = sorted(contours, key=lambda x: cv2.contourArea(x), reverse=True)
        baseContour = contours[0]

        peri = cv2.arcLength(baseContour, False)
        baseContour = cv2.approxPolyDP(baseContour, 0.05 * peri, False)

        print("Base area", cv2.contourArea(baseContour))
        for point in baseContour:
            cv2.circle(self.contourImg, (xCordStart + point[0][0], point[0][1]), 3, (255, 255, 0), cv2.FILLED)
        # cv2.drawContours(imgContour, baseContour, -1, , 5)

        print("len of cnt", len(baseContour))
        # print("printing base: ", baseContour)
        # get the y coordinates
        yCords = []
        for point in baseContour:
            yCords.append(point[0][1])
        yCords.sort()
        thirdQuartile = int(len(yCords) * .75)
        print("median", thirdQuartile, ":", yCords[thirdQuartile])

        worthyYCord = []
        # next get the base length
        for ypoint in yCords:
            # we want +-5% from 3rd quartile
            if yCords[thirdQuartile] * 1.05 > ypoint > yCords[thirdQuartile] * 0.95:
                worthyYCord.append(ypoint)
        print(worthyYCord)

        leadingBaseEdge = []
        for point in baseContour:
            if point[0][1] in worthyYCord:
                leadingBaseEdge.append(point)
        print(leadingBaseEdge)

        basexCords = []
        baseyCords = []
        for point in leadingBaseEdge:
            basexCords.append(point[0][0])
            baseyCords.append(point[0][1])
        minValue = min(basexCords)
        maxValue = max(basexCords)
        cv2.line(self.contourImg, pt1=(xCordStart + minValue, yCords[thirdQuartile]), pt2=(xCordStart + maxValue, yCords[thirdQuartile]),
                color=(0, 0, 255), thickness=2)

        xmin = xCordStart + minValue
        xmax =  xCordStart + maxValue
        self.robot_ycord = yCords[thirdQuartile]
        self.robot_midpoint = (xmin + xmax) //2
        print(xmin, self.robot_midpoint, xmax)

        baseDistance = dist.euclidean((xmin, self.robot_ycord), (xmax, self.robot_ycord))
        self.pixelsToWolrd = self.ROBOT_WIDTH / baseDistance # get pixle to meters ratio

    def get_world_coordinates(self, imagex, imagey):
        # Translate the origin to the robots center 
        worldx = self.pixelsToWolrd * (imagex - self.robot_midpoint)
        worldy = (self.ROBOT_CENTER_FROM_EDGE - self.robot_ycord * self.pixelsToWolrd) + (self.pixelsToWolrd * imagey)  
        print('pixle cords', imagex, imagey)
        print('world cords {}m, {}m'.format(round(worldx, 5),round(worldy,5)))

        # Simulation environment has different cooridante system then Gazebo environment
        if self.is_simulation:
            return worldx, worldy
        else:
            return worldy, worldx