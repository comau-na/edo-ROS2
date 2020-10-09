# e.DO Manual Control using ROS2

## Team Members
**Ayush Shrestha - Team Lead**  
**Scott Howard - Implementation Lead**  
**Adam Erdman - Integration Lead**  
**Seth Buchinger - Documentation Lead**  

## Goals / Objectives
* Convert wrapper class for e.DO robot previously developed by Jack Shelata
[Link to Jack's GitHub](https://github.com/jshelata/eDO_manual_ctrl)
* Confirm manual control capabilities on the COMAU e.DO robot using the NVIDIA Jetson control module

# edo-ROS2

This is a package that is currently being developed to create a wrapper class for the e.DO robot. The project will allow users to controll the e.DO directly from the command line.


# Dependencies

- ROS1 Melodic
- ROS2 Eloquent
- Installation of the ros1_bridge package
- edo_core_msgs package, along with the updated, ROS2 friendly, version of the package included in this repository
