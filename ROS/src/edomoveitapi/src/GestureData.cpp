#include "GestureData.h"
#include <string>

GestureData::GestureData(ros::NodeHandle& n_h){
  n = n_h;
  sub = n.subscribe("/gesture_class", 100,
      &GestureData::handPoseMsgCallback, this);
} 

void GestureData::handPoseMsgCallback(const std_msgs::String::ConstPtr& msg){
  gesture_class = msg->data.c_str();
}  

std::string GestureData::getGesture(){
  return gesture_class;
}  

