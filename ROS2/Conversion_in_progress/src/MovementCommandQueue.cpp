/** @file MovementCommandQueue.cpp
 *  @brief Class implementation for MovementCommandQueue. Used to send move
 *  commands to the e.DO with proper timing as the Tablet HMI does.
 *  @author Jack Shelata
 *  @date May 28, 2018
 */

#include "MovementCommandQueue.hpp"

/***************************************************************
**                Function(s) Definition
****************************************************************/

/** @brief Construct MovementCommandQueue object. Constructor creates and
 *  initializes a ROS Publisher and Subsciber to manage sending move commands
 *  to the e.DO and sets the resetCommand value.
 *  @param nh_in - ROS NodeHandle object by reference to create ROS Publishers
 *  and Subscribers
 *  @return MovementCommandQueue object
 *  @exception None
 */

//***DEBUG ROS1 to ROS2***
 /* MovementCommandQueue::MovementCommandQueue(ros::NodeHandle& nh_in) {
   nh = nh_in;
   move_ack_sub = nh.subscribe("/machine_movement_ack", 100,
       &MovementCommandQueue::moveAckCallback, this);
   move_ctrl_pub = nh.advertise<edo_core_msgs::MovementCommand>("/bridge_move",
       10,true);
   resetCommand.move_command = 67;
 } */

 MovementCommandQueue::MovementCommandQueue(rclcpp::Node& baseNode) {
   node = *baseNode;
   auto move_ack_sub = node->create_subscription("/machine_movement_ack", 100,
       &MovementCommandQueue::moveAckCallback, this);
   auto move_ctrl_pub = node->create_publisher<edo_core_msgs::msg:MovementCommand>("/bridge_move",
       10,true);
   resetCommand.move_command = 67;
 }
