import cv2
import numpy as np
from scipy.spatial import distance as dist
import imutils


# Reading an image

# img = cv2.imread("../Resources/lena.png")
# cv2.imshow("My pic", img)
# cv2.waitKey(0)

def stackImages(scale, imgArray):
    rows = len(imgArray)
    cols = len(imgArray[0])
    rowsAvailable = isinstance(imgArray[0], list)
    width = imgArray[0][0].shape[1]
    height = imgArray[0][0].shape[0]
    if rowsAvailable:
        for x in range(0, rows):
            for y in range(0, cols):
                if imgArray[x][y].shape[:2] == imgArray[0][0].shape[:2]:
                    imgArray[x][y] = cv2.resize(imgArray[x][y], (0, 0), None, scale, scale)
                else:
                    imgArray[x][y] = cv2.resize(imgArray[x][y], (imgArray[0][0].shape[1], imgArray[0][0].shape[0]),
                                                None, scale, scale)
                if len(imgArray[x][y].shape) == 2: imgArray[x][y] = cv2.cvtColor(imgArray[x][y], cv2.COLOR_GRAY2BGR)
        imageBlank = np.zeros((height, width, 3), np.uint8)
        hor = [imageBlank] * rows
        hor_con = [imageBlank] * rows
        for x in range(0, rows):
            hor[x] = np.hstack(imgArray[x])
        ver = np.vstack(hor)
    else:
        for x in range(0, rows):
            if imgArray[x].shape[:2] == imgArray[0].shape[:2]:
                imgArray[x] = cv2.resize(imgArray[x], (0, 0), None, scale, scale)
            else:
                imgArray[x] = cv2.resize(imgArray[x], (imgArray[0].shape[1], imgArray[0].shape[0]), None, scale, scale)
            if len(imgArray[x].shape) == 2: imgArray[x] = cv2.cvtColor(imgArray[x], cv2.COLOR_GRAY2BGR)
        hor = np.hstack(imgArray)
        ver = hor
    return ver


def getBase(imgSrc, imgContour):
    contours, hierarchy = cv2.findContours(imgSrc, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    worthyContours = []  # contains approximated contours
    yCordAverages = []  # contains indecies to the approximated contours
    for cnt in contours:
        if cv2.contourArea(cnt) > 600:
            peri = cv2.arcLength(cnt, False)
            approx = cv2.approxPolyDP(cnt, 0.05 * peri, False)
            worthyContours.append(approx)
            sum = 0
            for pnt in approx:
                sum += pnt[0][1]  # Add y value to sum
            yCordAverages.append(sum // len(approx))
    minIndex = yCordAverages.index(min(yCordAverages))
    baseContour = worthyContours[minIndex]
    print(cv2.contourArea(baseContour))
    cv2.drawContours(imgContour, baseContour, -1, (255, 255, 0), 5)

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
    cv2.line(imgContour, pt1=(minValue, yCords[thirdQuartile]), pt2=(maxValue, yCords[thirdQuartile]),
             color=(0, 0, 255), thickness=2)
    return minValue, maxValue, yCords[thirdQuartile]


def draw_circle(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDBLCLK:
        
        worldx = pixelsToWolrd * (x - midpoint)
        worldy = (0.230 - ycord*pixelsToWolrd) + (pixelsToWolrd * y)  
        print('pixle cords', x, y)
        print('world cords {}m, {}m'.format(round(worldx, 5),round(worldy,5)))
        # add half way to robot
        # translate backwards to robot origin
        # cv2.circle(img, (x, y), 100, (255, 0, 0), -1)

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
kernal = np.ones((4, 4), np.uint8)

img = cv2.imread("gazeboPose.png")
imgCnt = img.copy()

grayImg = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
blurImg = cv2.GaussianBlur(grayImg, (9, 9), 0)
cannyImg = cv2.Canny(blurImg, 150, 150)
dialatedImg = cv2.dilate(cannyImg, kernal, iterations=2)
eroidedImg = cv2.erode(dialatedImg, kernal, iterations=1)

xmin, xmax, ycord = getBase(eroidedImg, imgCnt)
midpoint = (xmin + xmax) //2
print(xmin,midpoint, xmax)

baseDistance = dist.euclidean((xmin, ycord), (xmax, ycord))
pixelsToWolrd = 0.270 / baseDistance # 27.0cm / base distance

imgStack = stackImages(0.9, ([img, eroidedImg, imgCnt]))
cv2.imshow("stack", imgStack)
cv2.namedWindow("clickme")
cv2.setMouseCallback("clickme", draw_circle)
cv2.imshow("clickme", imgCnt)

cv2.waitKey(0)
