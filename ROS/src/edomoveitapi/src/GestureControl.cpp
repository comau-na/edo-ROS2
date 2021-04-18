/** 
 *  ROS node created to modify Gazebo e.DO model robot pose using MoveIt.
 *  Movements are determined by gesture recognitition and classification 
 *  using NVIDIA deep learning models for hand pose estimation.
 *  @file GestureControl.cpp
 *  @author Laura Gipson
 *  @version 1.1 March 29, 2021
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Pose.h>
#include <string>

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
  if(gesture == "no hand")
  {
    pose.position.x = 0.397283811145;
    pose.position.y = -3.98077073662e-05;
    pose.position.z = 1.96567767916;
    pose.orientation.x = 1.84874275807e-06;
    pose.orientation.y = 0.382173407818;
    pose.orientation.z =-8.81235549789e-05;
    pose.orientation.w = 0.924090622497;

    pose_pub.publish(pose);
  }
  else if(gesture == "pan")
  {
    pose.position.x = 0.389834640181;
    pose.position.y = -5.13314643673e-05;
    pose.position.z = 1.48997718215;
    pose.orientation.x = 4.21981722855e-05;
    pose.orientation.y = 0.950104675016;
    pose.orientation.z = -3.84753687063e-05;
    pose.orientation.w = 0.311931247635;

    pose_pub.publish(pose);
  }
  else if(gesture == "stop")
  {
    pose.position.x = 9.88121415998e-05;
    pose.position.y = 5.92231270015e-09;
    pose.position.z = 2.12999999345;
    pose.orientation.x = -9.51910637009e-10;
    pose.orientation.y = 5.96880595398e-05;
    pose.orientation.z = 5.28870033485e-05;
    pose.orientation.w = 0.99999999682;

    pose_pub.publish(pose);
  }
  else if(gesture == "peace")
  {
    pose.position.x = 0.397283811145;
    pose.position.y = -3.98077073662e-05;
    pose.position.z = 1.96567767916;
    pose.orientation.x = 1.84874275807e-06;
    pose.orientation.y = 0.382173407818;
    pose.orientation.z =-8.81235549789e-05;
    pose.orientation.w = 0.924090622497;

    pose_pub.publish(pose);
  }
  else if (gesture == "fist")
  {
    pose.position.x = 0.294512422741;
    pose.position.y = -1.18905328599e-05;
    pose.position.z = 1.83634513545;
    pose.orientation.x = -1.52722533285e-05;
    pose.orientation.y = 0.706091448757;
    pose.orientation.z = -4.3907904711e-05;
    pose.orientation.w = 0.708120656267;

    pose_pub.publish(pose);
  }
  else //if (gesture == "ok")
  {
    pose.position.x = 0.77299087626;
    pose.position.y = -0.000307060946931;
    pose.position.z = 1.35325812948;
    pose.orientation.x = 0.000137034943648;
    pose.orientation.y = 0.708839058867;
    pose.orientation.z =  -0.000143836004292;
    pose.orientation.w = 0.705370221342;

    pose_pub.publish(pose);
  }
    // after pose executed, shut down node and kill pubs/subs
    ros::shutdown();
}

int main(int argc, char **argv)
{
  // initialize ROS and specify node name
  ros::init(argc, argv, "gesture_listener");

  // create handle for and initialize node 
  ros::NodeHandle n;

  // notify master Pose message to be published on edo_move topic
  pose_pub = n.advertise<geometry_msgs::Pose>("edo/edoMove", 100);

  // subscribe to ‘gesture_class’ topic and callback whenever new message arrives
  ros::Subscriber sub = n.subscribe("gesture_class", 100, handPoseMsgCallback);
  
  // loop message callbacks as fast as possible
  ros::spin();

  return 0;
}