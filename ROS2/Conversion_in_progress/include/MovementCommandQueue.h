/** @file MovementCommandQueue.h
 *  @brief Class definition for MovementCommandQueue
 *  @author Jack Shelata
 *  @date May 28, 2018
 */
#ifndef __MOVEMENT_COMMAND_QUEUE_H__
#define __MOVEMENT_COMMAND_QUEUE_H__

//#include <ros/ros.h>
#include <functional>
#include <memory>
#include "rclcpp/rclcpp.hpp"


#include "edo_core_msgs/msg/movement_command.hpp"
#include "edo_core_msgs/msg/movement_feedback.hpp"
#include <iostream>
#include <queue>
using std::placeholders::_1;
/** @brief Struct to hold information in the MovementCommandQueue class object
 */
struct MovementCommandQueueItem {
  edo_core_msgs::msg::MovementCommand message;   // MovementCommand to execute
  edo_core_msgs::msg::MovementFeedback status;   // MovementFeedback to store
};

/***************************************************************
**                Class(es) Definition
****************************************************************/

/** @brief This class manages the timing of sending move commands to the e.DO's
 *  "/bridge_move" ROS topic by subscribing to the "/machine_movement_ack" ROS
 *  topic
 */
class MovementCommandQueue : public rclcpp::Node{

public:

  // Class constructor takes in existing NodeHandle reference
  MovementCommandQueue();
  //: Node("movement_command_queue");
  // {
  //   move_ack_sub = this->create_subscription("/machine_movement_ack", 100,
  //     std::bind(&MovementCommandQueue::moveAckCallback, this, _1));
  //   move_ctrl_pub = this->create_publisher<edo_core_msgs::msg::MovementCommand>("/bridge_move",
  //     10,true, this, _1);
  //   resetCommand.move_command = 67;
  // }

  // Function to publish command to "/bridge_move"
  void sendMoveCommand(edo_core_msgs::msg::MovementCommand cmd);

  // Function to manage sending cancel command which must precede move commands
  void pushMoveCommand(edo_core_msgs::msg::MovementCommand cmd);

  // Callback function to manage queued commands based on MovementFeedback
  // messages from "/machine_movement_ack"
  // Mimics code found in ros.service.ts
  void moveAckCallback(const edo_core_msgs::msg::MovementFeedback::SharedPtr fb);

  bool stillRunning();

private:

  //rclcpp::Node node;                                         // ROS node handle
  std::queue<MovementCommandQueueItem> pendingQueue;          // Pending msg q
  std::queue<MovementCommandQueueItem> waitingReceiveQueue;   // Received msg q
  std::queue<MovementCommandQueueItem> waitingExecutedQueue;  // Executed msg q
  rclcpp::Subscription<edo_core_msgs::msg::MovementCommand>::SharedPtr move_ack_sub;
  rclcpp::Publisher<edo_core_msgs::msg::MovementCommand>::SharedPtr move_ctrl_pub;
  //ros::Subscriber move_ack_sub;                               //
  //ros::Publisher move_ctrl_pub;                               //
  edo_core_msgs::msg::MovementCommand resetCommand;                // Reset Command

};


#endif
