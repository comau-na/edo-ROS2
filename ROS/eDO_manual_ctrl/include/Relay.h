/** @file Relay.h
 *  @brief Class definition for Relay
 *  @author Jack Shelata
 *  @date August 2, 2018
 */ 
#ifndef __RELAY_H__
#define __RELAY_H__

#include <ros/ros.h>
#include "MovementCommandQueue.h"
#include "StateChecker.h"
#include "edo_core_msgs/MachineState.h"
#include "edo_core_msgs/MovementCommand.h"
#include <vector>
#include <std_msgs/Int8MultiArray.h>

/***************************************************************
**                Class(es) Definition
****************************************************************/

/** @brief This class relays joint angles published by simulation over
 *  the "/relay" topic to the e.DO robot
 */
class Relay {

public:
  
  // Class constructor takes in existing NodeHandle by reference
  Relay(ros::NodeHandle& nh_in, bool quick);

  // Class destructor destroys Relay object properly
  ~Relay();

  // Callback function to get Int8MultiArray from "/relay" topic
  void relayCallback(const std_msgs::Int8MultiArray& msg);

  // Getter member funciton to check if move_ctrl is still runnint
  bool getStillRunning();

private:

  ros::NodeHandle       nh;             // To save NodeHandle  
  ros::Subscriber       relaySub;       // ROS Subscriber gets sequence from Sim
  MovementCommandQueue* move_ctrl;      // Handles sending moves to e.DO
  bool                  quick;          // If true, sends point of contact and
                                        // reset only
};


#endif
