/** @file StateChecker.cpp
 *  @brief Class implementation for StateChecker. Used to check the state of
 *  e.DO
 *  @author Scott Howard, based on work by Jack Shelata
 *  @date 10/15/2020
 */

#include "StateChecker.h"

// ===============================================================
// Constructor for StateChecker. Inherits from rclcpp Node.
// Simply creates the subscription.
// ===============================================================
  StateChecker::StateChecker() : Node("machine_state")
  {
    subscription_ = this->create_subscription<edo_core_msgs::msg::MachineState>(
      "/machine_state", 10, std::bind(&StateChecker::stateCallback, this, _1));
  }


// ===============================================================
// Callback function to get and save state number from "/machine_state".
// It also sets the stateReceived variable to true to indicate that
// a machine state has been received.
// ===============================================================

 void StateChecker::stateCallback(const edo_core_msgs::msg::MachineState::SharedPtr msg) 
  {
    std::cout << "MachineState: " << (uint) msg->current_state << "\n";
    
    stateReceived = true;
    machineState = msg->current_state;

  }
  

// ===============================================================
// Getter member function to return the saved machine state number.
// ===============================================================
int StateChecker::getState(){return machineState;}

// ===============================================================
// Getter member function to return the stateReceived bool
// ===============================================================
int StateChecker::getStateReceived(){return stateReceived;}


/*
ROS1 code
StateChecker::StateChecker(ros::NodeHandle& nh_in){
  nh = nh_in;
  machine_state_sub = nh.subscribe("/machine_state", 10,
      &StateChecker::stateCallback, this);
  stateReceived = false;
}  // StateChecker::StateChecker()

void StateChecker::stateCallback(const edo_core_msgs::MachineState& state){
  stateReceived = true;
  machine_state = state.current_state;
}  // StateChecker::stateCallback()



*/











