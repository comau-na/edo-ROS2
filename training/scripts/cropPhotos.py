import xml.etree.ElementTree as ET
import os
import re
from PIL import Image

imageFolder = "C:\\Users\\Khalid\\Downloads\\cropPhotos\\JPEGImages"
annotationPath = "C:\\Users\\Khalid\\Downloads\\cropPhotos\\Annotations"
savePath = "C:\\Users\\Khalid\\Downloads\\cropPhotos\\classes"
files = os.listdir(annotationPath)

print(len(files), " files found")

# Then go to annotations
for file in files:
    editFilePath = os.path.join(annotationPath, file)
    editTree = ET.parse(editFilePath)
    editRoot = editTree.getroot()

    photoName = ""
    image = None
    for xmlObjs in editRoot.iter():
        if xmlObjs.tag == "filename":
            photoName = xmlObjs.text
            print("Reading photo", photoName)
            imagePath = os.path.join(imageFolder, photoName)
            image = Image.open(imagePath)
    # Go through every object and grap cords
    xmin, ymin, xmax, ymax = 0, 0, 0, 0

    objectName = ""
    for xmlObject in editRoot.iter("object"):
        for childObj in xmlObject.iter():
            #print("tag, ", childObj.tag, "text: ", childObj.text)
            if childObj.tag == "name":
                objectName = childObj.text
            if childObj.tag == "xmin":
                xmin = childObj.text
            if childObj.tag == "ymin":
                ymin = childObj.text
            if childObj.tag == "xmax":
                xmax = childObj.text
            if childObj.tag == "ymax":
                ymax = childObj.text
        print("creating ", objectName, " with ", xmin, ymin, xmax, ymax)

        xmin = int(float(xmin))
        ymin = int(float(ymin))
        xmax = int(float(xmax))
        ymax = int(float(ymax))
        cords = [xmin,ymin,xmax,ymax]
        width, height = image.size

        # for i in range(len(cords)):
        #     if i % 2 == 0 and 0 < cords[i] * 1.1 < width:
        #         print(cords[i], "-> ", end="")
        #         cords[i] = int(cords[i] * 1.1)
        #         print(cords[i])
        #     elif 0 < cords[i] * 1.1 < height:
        #         print(cords[i], "-> ", end="")
        #         cords[i] = int(cords[i] * 1.1)
        #         print(cords[i])
        
        # Add padding to the image
        cords[0] = int(cords[0] * 0.98)
        cords[1] = int(cords[1] * 0.98)
        cords[2] = int(cords[2] * 1.02)
        cords[3] = int(cords[3] * 1.02)

        print("creating ", objectName, " with ", cords)

        # for cord in cords:
        #     print(cord)
        crpImg = image.crop((cords[0], cords[1], cords[2], cords[3]))
        croppedImagePath = os.path.join(savePath, objectName, objectName+photoName)
        crpImg.save(croppedImagePath)
        #print("image size: ", crpImg.size, " -> ", end="")
