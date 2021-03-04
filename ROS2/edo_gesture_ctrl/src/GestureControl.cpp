#include <functional>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <iostream>

using std::placeholders::_1;

class GestureSubscriber : public rclcpp::Node
{
	public: GestureSubscriber() : Node("class_sub")
	{
		subscription_ = this->create_subscription<std_msgs::msg::String>(
		"gesture_class", 100 std::bind(&GestureSubscriber::class_callback, this, _1));
	}
	private: void class_callback(const std_msgs::msg::String::SharedPtr msg) const
	{
		RCLCPP_INFO(this->get_logger(), msg->data.c_str());
	}
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

/* TO DO:
	Add functions to initialize movement commands/movement queue

*/
}

