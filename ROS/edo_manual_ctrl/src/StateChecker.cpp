/** @file StateChecker.cpp
 *  @brief Class implementation for StateChecker. Used to check the state of
 *  e.DO
 *  @author Scott Howard, based on work by Jack Shelata
 *  @date 10/15/2020
 */

#include "StateChecker.h"
//#include <memory>
//using std::placeholders::_1;


/***************************************************************
**                Function(s) Definition
****************************************************************/

/** @brief Construct StateChecker object. Constructor creates and initializes
 *  subscriber to check the e.DO's Machine State
 *  @param nh_in - ROS NodeHandle object by reference to create ROS Subscriber
 *  @return StateChecker object
 *  @exception None
 */
  StateChecker::StateChecker(std::shared_ptr<rclcpp::Node> node) //: Node("machine_state_sub")
  {
    subscription_ = node->create_subscription<edo_core_msgs::msg::MachineState>(
      "/machine_state", 10, std::bind(&StateChecker::stateCallback, this, _1));
  }

/** @brief Callback function to get and save state number from "/machine_state"
 *  ROS topic.
 *  @param state - MachineState message type from "/machine_state" ROS topic
 *  @return void
 *  @exception None
 */

 void StateChecker::stateCallback(const edo_core_msgs::msg::MachineState::SharedPtr msg) 
  {
    std::cout << "MachineState: " << msg->current_state << "\n";
    //RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->current_state);
     stateReceived = true;
    machineState = msg->current_state;

  }
  
/** @brief Getter member function to return the saved machine state number.
 *  @param None
 *  @return int - 
 *  @exception None
 */

int StateChecker::getState(){return machineState;}
/** @brief Getter member function to return the stateReceived bool
 *  @param None
 *  @return bool - Value of stateReceived (true if state has been received)
 *  @exception None
 */

int StateChecker::getStateReceived(){return stateReceived;}


















// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "edo_core_msgs/msg/machine_state.hpp"

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
      subscription_ = this->create_subscription<edo_core_msgs::msg::MachineState>(
        "machine_state", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }


  private:
    void topic_callback(const edo_core_msgs::msg::MachineState::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->current_state);
    }
    rclcpp::Subscription<edo_core_msgs::msg::MachineState>::SharedPtr subscription_;
   
  };

int main(int argc, char * argv[])
{
  edo_core_msgs::msg::MachineState msg;
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
*/