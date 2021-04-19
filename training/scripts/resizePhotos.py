import xml.etree.ElementTree as ET
import os
import re
from PIL import Image

imageFolder = "C:\\Users\\Khalid\\Downloads\\jetson-inference\\python\\training\\classification\\data\\edo"
files = os.listdir(imageFolder)

for root, dirs, files in os.walk(imageFolder):
    print("loaded ", len(files), " files from ", root)
    for file in files:
        imagePath = os.path.join(root, file)
        image = Image.open(imagePath)
        print("image size: ", image.size, " -> ", end="")
        image.thumbnail((800, 800))
        image.save(imagePath)
        print(image.size)

# Start with renaming images
# print("loaded ", len(files), " files from ", imageFolder)
