'''Copyright (c) 2019-2020, NVIDIA CORPORATION. All rights reserved.

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.'''


# This creates a node which will subscribe to the Image topic, 
# perform inference using PyTorch and send the results of the image
# on Classification2D in vision_msgs


# make the nedded ROS imports
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from classification_service.srv import GetClassification

# We will be publishing on vision_msgs
from vision_msgs.msg import Classification2D, ObjectHypothesis

# Import necessary PyTorch and related frameworks
import torch
import torchvision
from torchvision import models
from torchvision import transforms
from PIL import Image
import numpy as np
from timeit import default_timer as timer

import os

import cv2
from cv_bridge import CvBridge, CvBridgeError

class WebcamClassifier(Node):

    def __init__(self):
        super().__init__('webcam_classification')
        # Create a subscriber to the Image topic
        self.image_subscriber = self.create_service(GetClassification, 'classify_image', self.listener_callback)

        # create a publisher onto the vision_msgs 2D classification topic

        # create a model parameter, by default the model is resnet18
        self.declare_parameter('model', "resnet18")
        model_name = self.get_parameter('model')
        
        print(model_name.value)

        # Use the SqueezeNet model for classification
        self.classification_model = self.create_classification_model(model_name)

        # Load it onto the GPU and set it to evaluation mode
        self.classification_model.eval().cuda()

        # Use CV bridge to convert ROS Image to CV_image for visualizing in window
        self.bridge = CvBridge()

        # Find the location of the ImageNet labels text and open it
        with open(os.getenv("HOME") + '/ros2_models/classlabels.txt') as f:
           self.labels = [line.strip() for line in f.readlines()]
           print("Labels loaded:", self.labels)
 
    

    def create_classification_model(self, model_name):
        
        if(str(model_name.value) == "resnet18"):
            model_path = os.getenv("HOME") + '/ros2_models/model_best.pth'
            sd = torch.load(model_path)
            model = models.resnet18()
            model.fc = torch.nn.Linear(512,6)
            model.load_state_dict(sd['state_dict'])
            return model

        print("Only valid model option currently is resnet18")


    def classify_image(self,img):
        
        transform = transforms.Compose([
            transforms.Resize(256),
            transforms.CenterCrop(224),
            transforms.ToTensor(),
            transforms.Normalize
                ([0.485, 0.456, 0.406],
                [0.229, 0.224, 0.225])
        ])
        # tensor_to_image = transforms.ToPILImage()
        # img = tensor_to_image(img)
        #img_t = transform(img).cuda()
        batch_t = transform(img).cuda().unsqueeze_(0)
	
        # Classify the image
        start = timer() 
        out = self.classification_model(batch_t)
        end = timer()

        print("Live classifier time: ", (end-start))

        _, index = torch.max(out, 1)

        percentage = torch.nn.functional.softmax(out, dim=1)[0] * 100

        return self.labels[index[0]] , percentage[index[0]].item()
        

    def listener_callback(self, request, response):
         # Use OpenCV to visualize the images being classified from webcam 
        start = timer()

        try:
            cv_image = self.bridge.imgmsg_to_cv2(request.img, "bgr8")
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            im_pil = Image.fromarray(cv_image)
        except CvBridgeError as e:
            print(e)

        # img_data = np.asarray(request.img.data)
        # img_reshaped = np.reshape(img_data,(request.img.height, request.img.width, 3))
        classified, confidence = self.classify_image(im_pil)
        end = timer()
        time = str(end-start)
        to_display = "Classification: " + classified + " ,confidence: " + str(confidence) + " time: " + time
        self.get_logger().info(to_display) 

        # Definition of Classification2D message
        response.classification.header = request.img.header
        result = ObjectHypothesis()
        result.id = classified
        result.score = confidence
        response.classification.results.append(result)
        print('Returning the response')
        #cv2.imwrite(os.path.join('/home/khalid/Downloads/temp/cvimages', classified + str(confidence)) + '.jpeg', cv_image)
      
        # Use OpenCV to visualize the images being classified from webcam 
        # try:
        #   cv_image = self.bridge.imgmsg_to_cv2(request.img, "bgr8")
        #   cv2.imwrite(os.path.join('/home/khalid/Downloads/temp/cvimages', classified + str(confidence)) + '.jpeg', cv_image)
        # except CvBridgeError as e:
        #   print(e)
        # cv2.imshow('webcam_window', cv_image)
        # cv2.waitKey(0)

        # Publish Classification results
        return response
