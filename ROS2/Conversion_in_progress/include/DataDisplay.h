
/** @file DataDisplay.h
 *  @brief Class definiton for DataDisplay - converted for RO2
 *  @date October 8, 2020
 *  @author Seth Buchinger - based on Jack Shelata ROS1 class

*/


#ifndef __DATA_DISPLAY_H__
#define __DATA_DISPLAY_H__

//#include <ros/ros.h>
#include <functional>
#include <memory>
#include "rclcpp/rclcpp.hpp" //ros2
//#include "edo_core_msgs/CartesianPose.h"
#include "edo_core_msgs/CartesianPose.hpp"
//#include "edo_core_msgs/MachineState.h"
#include "edo_core_msgs/MachineState.hpp"
//#include "edo_core_msgs/JointStateArray.h"
#include "edo_core_msgs/JointStateArray.hpp"
#include <iostream>
using std::placeholder::_1

/***************************************************************
**                Class(es) Definition
****************************************************************/

/** @brief This class creates and stores ROS2 Subscribers to
 *  output the e.DO's Cartesian Position, Machine State, and Joint Angle Data.
 */
class DataDisplay : public rclcpp::Node {
  
public:

  // Constructor creates and initializes subscribers and bools for checking
  // for completion
  // DataDisplay(ros::NodeHandle& nh_in);

  DataDisplay(std::shared_ptr<rclcpp::Node>& baseNode);

  // Callback function to print CartesianPose message 
  //void printPoseData(const edo_core_msgs::CartesianPose& pose);
  void printPoseData(const edo_core_msgs::msg::CartesianPose& pose);

  // Callback function to print MachineState message
  //void printState(const edo_core_msgs::MachineState& state);
  void printState(const edo_core_msgs::msg::MachineState& state);

  // Callback function to print JointStateArray message
  // void printJointPose(const edo_core_msgs::JointStateArray& pose);
  void printJointPose(const edo_core_msgs::msg::JointStateArray& pose);

  // Member function to tell whether cartesian data has been printed
  bool getCartesianPrinted();

  // Member function to tell whether cartesian data has been printed
  bool getStatePrinted();

  // Member function to tell whether joint data has been printed
  bool getJointPrinted();

private:

  //ros::NodeHandle nh;                                 // ROS Node Handle
  rclcpp::Node node;
  //ros::Subscriber cartesian_pose_sub;                 // ROS subscriber
  ros::Subscription<edo_core_msgs::msg::CartesianPose>::SharedPtr cartesian_pose_sub;                 // ROS subscriber
  //ros::Subscriber machine_state_sub;                  // ROS subscriber
  ros::Subscription<edo_core_msgs::msg::MachineState>::SharedPtr machine_state_sub;                  // ROS subscriber
  //ros::Subscriber joint_pose_sub;                     // ROS subscriber
  ros::Subscription<edo_core_msgs::msg::JointStateArray>::SharedPtr joint_pose_sub;                     // ROS subscriber

  bool cartesianPrinted, statePrinted, jointPrinted;  // True when printed
  
};


#endif
