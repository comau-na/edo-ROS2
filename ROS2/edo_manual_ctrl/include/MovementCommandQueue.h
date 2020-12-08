//LEGACY INFORMATION
/** @file MovementCommandQueue.h
 *  @brief Class definition for MovementCommandQueue
 *  @author Jack Shelata
 *  @date May 28, 2018
 */
//END LEGACY information


 //ROS2 Migration Performed by Adam Erdman
 //Date 8 DEC 2020

#ifndef __MOVEMENT_COMMAND_QUEUE_H__
#define __MOVEMENT_COMMAND_QUEUE_H__


#include <functional>
#include <memory>
#include "rclcpp/rclcpp.hpp"


#include "edo_core_msgs/msg/movement_command.hpp"
#include "edo_core_msgs/msg/movement_feedback.hpp"
#include <iostream>
#include <queue>
using std::placeholders::_1;

/** Struct to hold information in the MovementCommandQueue class object
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


  std::queue<MovementCommandQueueItem> pendingQueue;          // Pending msg q
  std::queue<MovementCommandQueueItem> waitingReceiveQueue;   // Received msg q
  std::queue<MovementCommandQueueItem> waitingExecutedQueue;  // Executed msg q
  rclcpp::Subscription<edo_core_msgs::msg::MovementFeedback>::SharedPtr move_ack_sub;
  rclcpp::Publisher<edo_core_msgs::msg::MovementCommand>::SharedPtr move_ctrl_pub;

  edo_core_msgs::msg::MovementCommand resetCommand;                // Reset Command
  rclcpp::TimerBase::SharedPtr timer_;



};


#endif


/*=============================================================
Legacy code start
===============================================================*/

// #ifndef __MOVEMENT_COMMAND_QUEUE_H__
// #define __MOVEMENT_COMMAND_QUEUE_H__
//
// #include <ros/ros.h>
// #include "edo_core_msgs/MovementCommand.h"
// #include "edo_core_msgs/MovementFeedback.h"
// #include <iostream>
// #include <queue>
//
// /** @brief Struct to hold information in the MovementCommandQueue class object
//  */
// struct MovementCommandQueueItem {
//   edo_core_msgs::MovementCommand message;   // MovementCommand to execute
//   edo_core_msgs::MovementFeedback status;   // MovementFeedback to store
// };
//
// /***************************************************************
// **                Class(es) Definition
// ****************************************************************/
//
// /** @brief This class manages the timing of sending move commands to the e.DO's
//  *  "/bridge_move" ROS topic by subscribing to the "/machine_movement_ack" ROS
//  *  topic
//  */
// class MovementCommandQueue {
//
// public:
//
//   // Class constructor takes in existing NodeHandle reference
//   MovementCommandQueue(ros::NodeHandle& nh_in);
//
//   // Function to publish command to "/bridge_move"
//   void sendMoveCommand(edo_core_msgs::MovementCommand cmd);
//
//   // Function to manage sending cancel command which must precede move commands
//   void pushMoveCommand(edo_core_msgs::MovementCommand cmd);
//
//   // Callback function to manage queued commands based on MovementFeedback
//   // messages from "/machine_movement_ack"
//   // Mimics code found in ros.service.ts
//   void moveAckCallback(const edo_core_msgs::MovementFeedback& fb);
//
//   bool stillRunning();
//
// private:
//
//   ros::NodeHandle nh;                                         // ROS node handle
//   std::queue<MovementCommandQueueItem> pendingQueue;          // Pending msg q
//   std::queue<MovementCommandQueueItem> waitingReceiveQueue;   // Received msg q
//   std::queue<MovementCommandQueueItem> waitingExecutedQueue;  // Executed msg q
//   ros::Subscriber move_ack_sub;                               //
//   ros::Publisher move_ctrl_pub;                               //
//   edo_core_msgs::MovementCommand resetCommand;                // Reset Command
//
// };
//
//
// #endif


/*=============================================================
Legacy code end
===============================================================*/
