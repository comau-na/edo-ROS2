
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
class StateChecker : public rclcpp::Node
{
public:
  StateChecker()
  : Node("machine_state_sub")
  {
    subscription_ = this->create_subscription<edo_core_msgs::msg::MachineState>(
      "/machine_state", 10, std::bind(&StateChecker::stateCallback, this, _1));
  }
  int getState(){return machineState;}
  int getStateReceived(){return stateReceived;}

private:
  void stateCallback(const edo_core_msgs::msg::MachineState::SharedPtr msg) 
  {
    std::cout << "MachineState: " << msg->current_state << "\n";
    //RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->current_state);
     stateReceived = true;
    machineState = msg->current_state;

  }
  
  rclcpp::Subscription<edo_core_msgs::msg::MachineState>::SharedPtr subscription_;
  uint machineState;
  bool stateReceived = false;
};



#endif
