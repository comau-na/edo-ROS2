import xml.etree.ElementTree as ET
import os
import re

imagePath = "C:/Users/Khalid/Downloads/onlyv2/testing/images"  # C:\Users\Khalid\Downloads\ImageV2\Sunny
annotationPath = "C:/Users/Khalid/Downloads/onlyv2/testing/annotations"  # C:\Users\Khalid\Downloads\ImageV2\Sunny
appendOwner = "wam"
files = os.listdir(imagePath)

# Start with renaming images
print("loaded ", len(files), " files from ", imagePath)
for file in files:
    if "Screenshot" in file:
    # if file[0].isdigit():
        originalFilepath = os.path.join(imagePath, file)
        renamedFile = file.replace(' ', '_')
        modifiedFilePath = os.path.join(imagePath, renamedFile)
        os.rename(originalFilepath, modifiedFilePath)
        print(file, " -> ", renamedFile)

files = os.listdir(annotationPath)
print("loaded ", len(files), " files from ", annotationPath)
# Then go to annotations
for file in files:
    # if file[0].isdigit():
    if "Screenshot" in file:
        originalFilepath = os.path.join(annotationPath, file)
        renamedFile = file.replace(' ', '_')
        modifiedFilePath = os.path.join(annotationPath, renamedFile)
        os.rename(originalFilepath, modifiedFilePath)

        mytree = ET.parse(modifiedFilePath)
        myroot = mytree.getroot()

        renameItems = ['filename']

        # for editItem in renameItems:
        #     for xmlItem in myroot.iter(editItem):
        #         # updates the price value
        #         index = xmlItem.text.rfind(re.search(r'\d+', xmlItem.text).group())
        #         xmlItem.text = xmlItem.text[ : index] + appendOwner + xmlItem.text[index : ]
        #         break

        for editItem in renameItems:
            for xmlItem in myroot.iter(editItem):
                # updates the price value
                print(xmlItem.text, " -> ", end="")
                xmlItem.text = xmlItem.text.replace(' ', '_')
                print(xmlItem.text)
                break

        mytree.write(modifiedFilePath)
