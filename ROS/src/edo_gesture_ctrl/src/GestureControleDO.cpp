/** 
 *  ROS node created to modify physical e.DO robot pose using MoveIt.
 *  Movements are determined by gesture recognitition and classification 
 *  using NVIDIA deep learning models for hand pose estimation.
 *  @file GestureControleDO.cpp
 *  @author Laura Gipson
 *  @version 1.0 April 15, 2021
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Pose.h>
#include <string>
#include <unistd.h>

ros::Publisher pose_pub;
geometry_msgs::Pose pose;


/** 
 *  Called when a new message has arrived on the gesture_class topic.
 *  @param msg ROS std_msgs/String with classification data.
 *  @return void
 */

void handPoseMsgCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("The Position of the hand is : [%s]", msg->data.c_str());
  std::string gesture = msg->data.c_str();
  
  // conditional statement decides which pose to publish based on gesture
  if(gesture == "pan")
  {
    pose.position.x = 0.267723;
    pose.position.y = 2.627721;
    pose.position.z = 0.373511;
    pose.orientation.x = -4.91134E-05;
    pose.orientation.y = 1;
    pose.orientation.z = 4.63919E-05;
    pose.orientation.w = -0.000838247;

    pose_pub.publish(pose);
  }
  else if(gesture == "stop")
  {
    pose.position.x = -7.60974e-05;
    pose.position.y = -4.86276e-09;
    pose.position.z = 0.99;
    pose.orientation.x = 4.79277e-10;
    pose.orientation.y = -7.52148e-05;
    pose.orientation.z = 5.83505e-05;
    pose.orientation.w = 1;

    pose_pub.publish(pose);
  }
  else if(gesture == "peace")
  {
    pose.position.x = 0.312586;
    pose.position.y = -2.06157e-05;
    pose.position.z = 0.860698;
    pose.orientation.x = 2.27703e-05;
    pose.orientation.y = 0.382252;
    pose.orientation.z = -5.89856e-06;
    pose.orientation.w = 0.924056;

    pose_pub.publish(pose);
  }
  else if (gesture == "fist")
  {
    pose.position.x = 0.17457;
    pose.position.y = -2.53597E-05;
    pose.position.z = 0.814852;
    pose.orientation.x = 8.563771E-05;
    pose.orientation.y = 0.708417;
    pose.orientation.z = -1.72335E-05;
    pose.orientation.w = 0.705794;

    pose_pub.publish(pose);
  }
  else //if (gesture == "ok")
  {
    pose.position.x = 0.415784;
    pose.position.y = -2.10989e-05;
    pose.position.z = 0.131993;
    pose.orientation.x = -0.00891356;
    pose.orientation.y = 0.999957;
    pose.orientation.z =  -5.11101e-05;
    pose.orientation.w = 0.00243461;

    pose_pub.publish(pose);
  }
    // after pose executed, shut down node and kill pubs/subs
    //ros::shutdown();

    // wait for pose to execute before callback new message
    usleep(11000000);
}

int main(int argc, char **argv)
{
  // initialize ROS and specify node name
  ros::init(argc, argv, "gesture_listener");

  // create handle for and initialize node 
  ros::NodeHandle n;

  // notify master Pose message to be published on edo_move topic
  pose_pub = n.advertise<geometry_msgs::Pose>("edo/edo_move", 100);

  // subscribe to ‘gesture_class’ topic and callback whenever new message arrives
  ros::Subscriber sub = n.subscribe("gesture_class", 100, handPoseMsgCallback);
  
  // loop message callbacks as fast as possible
  ros::spin();

  return 0;
}
