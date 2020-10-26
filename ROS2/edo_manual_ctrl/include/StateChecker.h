
/** @file StateChecker.h
 *  @brief Class definition for StateChecker
 *  @author Scot Howard, based on work by Jack Shelata
 *  @date 10/15/2020
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
class StateChecker : public rclcpp::Node
{
public:
  StateChecker(); //std::shared_ptr<rclcpp::Node> node
  
  int getState();
  int getStateReceived();

private:
  void stateCallback(const edo_core_msgs::msg::MachineState::SharedPtr msg);
  
  rclcpp::Subscription<edo_core_msgs::msg::MachineState>::SharedPtr subscription_;
  uint machineState;
  bool stateReceived = false;
};



#endif
