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

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "edo_core_msgs/msg/movement_command.hpp"
#include "edo_core_msgs/msg/joint_calibration.hpp"
#include "edo_core_msgs/msg/joint_reset.hpp"
#include "edo_core_msgs/msg/joint_init.hpp"
using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0)
  {
    
    publisher_ = this->create_publisher<edo_core_msgs::msg::MovementCommand>("/bridge_move", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);
      edo_core_msgs::msg::MovementCommand msg;

       msg.move_command = 77;
    msg.move_type = 74;
    msg.ovr = 100;
    msg.delay = 1;
    msg.target.data_type = 74;
    msg.target.joints_mask = 63;
    msg.target.joints_data.resize(6, 0.0);

 std::cout << "joint angles as follows and press enter: J1 J2 J3 J4 J5 J6\n";

    for(int x = 0; x < 6; ++x){
        scanf("%f", &msg.target.joints_data[x]);
      }

    publisher_->publish(msg);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<edo_core_msgs::msg::MovementCommand>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
