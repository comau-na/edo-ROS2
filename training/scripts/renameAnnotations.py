# import xml.etree.ElementTree as ET
# import os
# import re
#
# sourcePath = "C:/Users/Khalid/Downloads/ImageV2/Annotations"  # C:\Users\Khalid\Downloads\ImageV2\Sunny
# editPath = "C:/Users/Khalid/Downloads/objs/onlyimagev2/Annotations"  # C:\Users\Khalid\Downloads\ImageV2\Sunny
# files = os.listdir(sourcePath)
#
# # Then go to annotations
# for file in files:
#
#     sourceFilePath = os.path.join(sourcePath, file)
#     sourceTree = ET.parse(sourceFilePath)
#     sourceRoot = sourceTree.getroot()
#
#     editFilePath = os.path.join(editPath, file)
#     editTree = ET.parse(editFilePath)
#     editRoot = editTree.getroot()
#
#     for xmlItem in editRoot.iter("object"):
#         # updates the price value
#         editRoot.remove(xmlItem)
#         break
#
#     for xmlItem in sourceRoot.iter("object"):
#         editRoot.append(xmlItem)
#
#     editTree.write(editFilePath)

import xml.etree.ElementTree as ET
import os
import re

editPath = "C:\\Users\\Khalid\\Downloads\\jetson-inference\\python\\training\\detection\\ssd\\data\\edo\\Annotations"
# C:\Users\Khalid\Downloads\ImageV2\Sunny
files = os.listdir(editPath)
print(len(files), " files found")

wrongLabels = ['red_cube', 'blue_cube', 'blue_bucket', 'green_cube', 'green_bucket', 'red_bucket']
rightlabels = ['cube_red', 'cube_blue', 'bucket_blue', 'cube_green', 'bucket_green', 'bucket_red']

# Then go to annotations
for file in files:

    editFilePath = os.path.join(editPath, file)
    editTree = ET.parse(editFilePath)
    editRoot = editTree.getroot()

    for xmlItem in editRoot.iter("object"):
        for name in xmlItem.iter("name"):
            if name.text in wrongLabels:
                print(name.text, " -> ", end="")
                index = wrongLabels.index(name.text)
                name.text = rightlabels[index]
                print(name.text)

    editTree.write(editFilePath)
