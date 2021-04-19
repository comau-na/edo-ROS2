# e.DO Manual Control using ROS2 with NVIDIA AI Packages

## Team Members
-December 2020 Team

**Ayush Shrestha - Team Lead**  
**Scott Howard - Implementation Lead**  
**Adam Erdman - Integration Lead**  
**Seth Buchinger - Documentation Lead**  

-April 2021 Team

**Allen Paul - Team Lead**  
**Khalid Awad - Implementation Lead**  
**Laura Gipson - Documentation Lead**  
**Sunny Prajapati - QA Lead**   
**Wameed Miriako - Integration Lead**


## Goals / Objectives

* Convert wrapper class for e.DO robot previously developed by Jack Shelata
[Link to Jack's GitHub](https://github.com/jshelata/eDO_manual_ctrl)
* Confirm manual control capabilities on the COMAU e.DO robot using the NVIDIA Jetson control module


* Implement NVIDA AI packages with e.DO Robot in ROS2

# edo-ROS2

This package allows:
- users to control an e.DO robot from a linux command line terminal. 
- use gesture input to control the e.DO robot
- Environment object detection


# Dependencies

- ROS1 Melodic
- ROS2 Eloquent
- Installation of the ros1_bridge package
- edo_core_msgs package, along with the updated, ROS2 friendly, version of the package included in this repository
- ncurses
- Pytorch
- torchvision
- timm
- ros2_torch_trt: https://github.com/NVIDIA-AI-IOT/ros2_torch_trt
- vision_msgs: https://github.com/ros-perception/vision_msgs/tree/foxy

# Tutorials and Usage

There are tutorials to help set up this project located in the tutorials and documentation folder.
The following videos also provide examples of how to build and run this package.

**Building the package:** https://youtu.be/r9rDI4yP-Us

**Using the package:** https://youtu.be/egTKGkaJMBs

**Explanation of code base:** https://youtu.be/z64mXXk3P70

