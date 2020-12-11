
/** @file StateChecker.h
 *  @brief Class definition for StateChecker
 *  @author Scot Howard, based on work by Jack Shelata
 *  @date 10/15/2020
 */
#ifndef __STATE_CHECKER_H__
#define __STATE_CHECKER_H__


#include <functional>
#include <memory>
#include "rclcpp/rclcpp.hpp" 

#include "edo_core_msgs/msg/machine_state.hpp"
#include <iostream>
using std::placeholders::_1;


// ===============================================================
// This class creates a subscriber that listens for the machine state
// and stores the result
// ===============================================================
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

/*
//ROS1 code

class StateChecker {
  
public:
  // Cunstruct StateChecker object. Constructor creates and initializes ROS
  // Subscriber to check the e.DO's Machine State
  StateChecker(ros::NodeHandle& nh_in);

  // Callback function to get and save state number from "/machine_state"
  // ROS topic
  void stateCallback(const edo_core_msgs::MachineState& state);

  // Getter member function to return the saved machine state number
  int getState();
  
  // Getter member function to return the stateReceived bool
  bool getStateReceived();

private:

  ros::NodeHandle nh;                     // 
  ros::Subscriber machine_state_sub;      // 
  int machine_state;                      // 
  bool stateReceived;                     // 
  
};
*/
