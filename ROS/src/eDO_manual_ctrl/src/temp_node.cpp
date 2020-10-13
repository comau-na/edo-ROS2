#include <ros/ros.h>
#include "Relay.h"

int main(int argc, char **argv){

  ros::init(argc, argv, "relay");
  ros::NodeHandle nh;
  // Set second parameter to true for movement directly to contact point
  Relay relayObject(nh, false);
  ros::Rate loop_rate(100);

  while(ros::ok()){
    ros::spinOnce();
    loop_rate.sleep();
  }  


  return 0;
}  
