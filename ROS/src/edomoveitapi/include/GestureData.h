#ifndef __GESTURE_DATA_H__
#define __GESTURE_DATA_H__

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
#include <string>

extern std::string gesture;

class GestureData {
  
public:
  // Cunstruct GestureData object. Constructor creates and initializes ROS
  // Subscriber to get the Hand Pose (Gesture) classification
  GestureData(ros::NodeHandle& n_h);

  // Callback function to get and save gesture classification from "/gesture_class"
  // ROS topic
  void handPoseMsgCallback(const std_msgs::String::ConstPtr& msg);

  // Getter member function to return the gesture classification
  std::string getGesture();

private:
  ros::NodeHandle n;
  ros::Subscriber sub; 
  std::string gesture_class;
};

#endif
