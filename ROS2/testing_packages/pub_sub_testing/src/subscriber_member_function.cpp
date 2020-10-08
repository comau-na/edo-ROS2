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

#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "edo_core_msgs/msg/machine_state.hpp"

using std::placeholders::_1;

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
    stateReceived = true;
    machineState = msg->current_state;
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->current_state);

  }
  
  rclcpp::Subscription<edo_core_msgs::msg::MachineState>::SharedPtr subscription_;
  uint machineState;
  bool stateReceived = false;
};



void functionTest(){

std::cout << "lalalalala /n";
}

int main(int argc, char * argv[])
{
  

  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;
  auto subscriber_node = std::make_shared<StateChecker>();

  //rclcpp::spin(std::make_shared<MinimalSubscriber>());
  exec.add_node(subscriber_node);

  std::chrono::nanoseconds timeout;
  timeout= std::chrono::nanoseconds { 100 };

  int i = 0;
  while(i<5){
    exec.spin_once(timeout);
    std::cout << "spun \n";
    //std::this_thread::sleep_for(std::chrono::milliseconds(10));
    i++;
  }
  rclcpp::shutdown();
  return 0;
}
