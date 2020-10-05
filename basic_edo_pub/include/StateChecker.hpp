
/** @file StateChecker.h
 *  @brief Class definition for StateChecker
 *  @author Jack Shelata
 *  @date May 28, 2018
 */
#ifndef __STATE_CHECKER_H__
#define __STATE_CHECKER_H__


#include <functional>
#include <memory>
#include "rclcpp/rclcpp.hpp" //ros2
//#include <ros/ros.h>
#include "edo_core_msgs/msg/machine_state.hpp"
#include <iostream>
using std::placeholders::_1;
/***************************************************************
**                Class(es) Definition
****************************************************************/

/** @brief This class creates and stores a ROS Subscriber to get the state
 *  number of e.DO when the edo_manual_ctrl node is initially started
 */
class StateChecker : public rclcpp::Node {
  
public:
  // Cunstruct StateChecker object. Constructor creates and initializes ROS
  // Subscriber to check the e.DO's Machine State
  StateChecker::StateChecker() : Node("StateChecker"){
  //nh = nh_in;
  //machine_state_sub = nh.subscribe("/machine_state", 10,
  //  &StateChecker::stateCallback, this);

  machine_state_sub = this->create_subscription<edo_core_msgs::msg::MachineState>
    ("/machine_state",10, std::bind(&StateChecker::stateCallback, this, _1));

  stateReceived = false;
}  // StateChecker::StateChecker()

  // Callback function to get and save state number from "/machine_state"
  // ROS topic
  void stateCallback(const edo_core_msgs::msg::MachineState& state);

  // Getter member function to return the saved machine state number
  int getState();
  
  // Getter member function to return the stateReceived bool
  bool getStateReceived();

private:

  //ros::NodeHandle nh;                     // 
  rclcpp::Node nh;
  //ros::Subscriber machine_state_sub;      // 
  rclcpp::Subscription<edo_core_msgs::msg::MachineState>::SharedPtr machine_state_sub;
  int machine_state;                      // 
  bool stateReceived;                     // 
  
};


#endif
