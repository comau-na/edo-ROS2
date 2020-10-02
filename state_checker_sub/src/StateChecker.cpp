/** @file StateChecker.cpp
 *  @brief Class implementation for StateChecker. Used to check the state of
 *  e.DO
 *  @author Jack Shelata
 *  @date May 28, 2018
 */

#include "StateChecker.hpp"
//#include <memory>
using std::placeholders::_1;


/***************************************************************
**                Function(s) Definition
****************************************************************/

/** @brief Construct StateChecker object. Constructor creates and initializes
 *  subscriber to check the e.DO's Machine State
 *  @param nh_in - ROS NodeHandle object by reference to create ROS Subscriber
 *  @return StateChecker object
 *  @exception None
 */
StateChecker::StateChecker(){
  //nh = nh_in;
  //machine_state_sub = nh.subscribe("/machine_state", 10,
  //  &StateChecker::stateCallback, this);

  machine_state_sub = this->create_subscription<edo_core_msgs::msg::MachineState>
  	("/machine_state",10, std::bind(&StateChecker::stateCallback, this, _1));

  stateReceived = false;
}  // StateChecker::StateChecker()

/** @brief Callback function to get and save state number from "/machine_state"
 *  ROS topic.
 *  @param state - MachineState message type from "/machine_state" ROS topic
 *  @return void
 *  @exception None
 */
void StateChecker::stateCallback(const edo_core_msgs::msg::MachineState::SharedPtr state){
  stateReceived = true;
  machine_state = state.current_state;
}  // StateChecker::stateCallback()

/** @brief Getter member function to return the saved machine state number.
 *  @param None
 *  @return int - 
 *  @exception None
 */
int StateChecker::getState(){
  return
  machine_state;
}  // StateChecker::getState()

/** @brief Getter member function to return the stateReceived bool
 *  @param None
 *  @return bool - Value of stateReceived (true if state has been received)
 *  @exception None
 */
bool StateChecker::getStateReceived(){
  return stateReceived;
}  // StateChecker::getStateReceived()

int main(int argc, char * argv[])
{
  edo_core_msgs::msg::MachineState state;
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MachineState>());
  rclcpp::shutdown();
  return 0;
}