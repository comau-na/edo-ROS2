// Copyright 2015 Open Source Robotics Foundation, Inc.
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

#include <iostream>
#include <memory>
#include <utility>

// include ROS 1
#ifdef __clang__
# pragma clang diagnostic push
# pragma clang diagnostic ignored "-Wunused-parameter"
#endif
#include "ros/ros.h"
#include "std_msgs/String.h"

#include "edo_core_msgs/JointStateArray.h"
#include "edo_core_msgs/JointState.h"
#ifdef __clang__
# pragma clang diagnostic pop
#endif

// include ROS 2
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "edo_core_msgs/msg/joint_state_array.hpp"
#include "edo_core_msgs/msg/joint_state.hpp"


rclcpp::Publisher<edo_core_msgs::msg::JointStateArray>::SharedPtr pub;

void jointStateArrayCallback(const edo_core_msgs::JointStateArray::ConstPtr & ros1_msg)
{

	auto ros2_msg = std::make_unique<edo_core_msgs::msg::JointStateArray>();
  	ros2_msg->joints_mask = ros1_msg->joints_mask;
  	ros2_msg->joints.resize(6);
	int i = 0;
   for(edo_core_msgs::JointState joint : ros1_msg->joints)
    {
        
        ros2_msg->joints[i].position = joint.position;
		ros2_msg->joints[i].velocity = joint.velocity;
		ros2_msg->joints[i].current = joint.current;
		ros2_msg->joints[i].command_flag = joint.commandFlag;
		ros2_msg->joints[i].r_jnt = joint.R_jnt;         
		std::cout << "edo_core_msgs/msg/JointState[] joints: " << ros2_msg->joints[i].position << "\n";

        i++;
    }

 

  std::cout << "Passing along: [" << ros2_msg->joints_mask << "]" << std::endl;
  pub->publish(std::move(ros2_msg));
}

int main(int argc, char * argv[])
{
  // ROS 2 node and publisher
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("simple_1_to_2_bridge");
  pub = node->create_publisher<edo_core_msgs::msg::JointStateArray>("machine_algo_jnt_state_bridge", 10);

  // ROS 1 node and subscriber
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("machine_algo_jnt_state", 10, jointStateArrayCallback);

  ros::spin();

  return 0;
}
