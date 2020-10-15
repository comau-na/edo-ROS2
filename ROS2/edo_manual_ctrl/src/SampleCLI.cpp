//#include <ros/ros.h>
#include <functional>
#include <memory>
#include "rclcpp/rclcpp.hpp" //ros2

//#include "MovementCommandQueue.h"
//#include "DataDisplay.h"
#include "StateChecker.h"
#include "edo_core_msgs/msg/movement_command.hpp"
#include "edo_core_msgs/msg/joint_calibration.hpp"
#include "edo_core_msgs/msg/joint_reset.hpp"
#include "edo_core_msgs/msg/joint_init.hpp"

#include <thread>
#include <chrono>
#include <iostream>
#include <fstream>
#include <queue>
#include <string>
#include <iomanip>    // Used to set precision of output to 2 decimal places
//#include <ncurses.h>  // Used in the jog function to allow for keystrokes to
                      // be captured without an enter press

using namespace std::chrono_literals;


/** @brief Function handles initial callibration
 *  @param nh - ROS NodeHandle for creating Publishers for initializaiton,
 *  reset, and calibration
 *  @param recalib - bool is true if the e.DO has aleady been calibrated and
 *  is being recalibrated so that initialization and reset are not repeated
 *  @return void
 *  @exception None
*/
void calibrate(std::shared_ptr<rclcpp::Node> node, bool recalib){ //rclcpp::executors::SingleThreadedExecutor& exec
  
  
  //ros::Publisher calib_pub = nh.advertise<edo_core_msgs::JointCalibration>("/bridge_jnt_calib",10);
  auto calib_pub = node->create_publisher<edo_core_msgs::msg::JointCalibration>("/bridge_jnt_calib", 1000);
  rclcpp::executors::SingleThreadedExecutor exec;

  rclcpp::WallRate loop_rate(10);

  edo_core_msgs::msg::JointCalibration calib_msg;
  std::chrono::milliseconds timespan(10000);   // To sleep program for 10 sec
  

  char proceed = '\n';  // Char to allow user to control when commands are sent


  if(!recalib){

    //ros::Publisher reset_pub = nh.advertise<edo_core_msgs::JointReset>("/bridge_jnt_reset",10);
    auto reset_pub = node->create_publisher<edo_core_msgs::msg::JointReset>("/bridge_jnt_reset", 10);


    //ros::Publisher init_pub = nh.advertise<edo_core_msgs::JointInit>("/bridge_init",10);  
    auto init_pub = node->create_publisher<edo_core_msgs::msg::JointInit>("/bridge_init", 10);


    edo_core_msgs::msg::JointReset reset_msg;
    edo_core_msgs::msg::JointInit init_msg;

    while(proceed != 'y'){
      std::cout << "Enter 'y' to initialize 6-Axis eDO w/o gripper: ";
      std::cin >> proceed;
    }
    proceed = '\n';                             // Reset char for next prompt
    init_msg.mode = 0;
    init_msg.joints_mask = 63;
    init_msg.reduction_factor = 0.0;
    while(init_pub->get_subscription_count() == 0){
      loop_rate.sleep();
      std::cout << "checking for Subscribers to /bridge_init... \n";
    }
    init_pub->publish(init_msg);
    //ros::spinOnce();
    //loop_rate.sleep();
  
    std::this_thread::sleep_for(timespan);      // while e.DO initializes

    while(proceed != 'y'){
      std::cout << "Enter 'y' to disengage brakes: ";
      std::cin >> proceed;
    }  
    proceed = '\n';                             // Reset char for next prompt
    reset_msg.joints_mask = 63;
    reset_msg.disengage_steps = 2000;
    reset_msg.disengage_offset = 3.5;
    while(reset_pub->get_subscription_count() == 0){
      loop_rate.sleep();
      std::cout << "Checking for Subscribers to /bridge_jnt_reset... \n";
    }
    reset_pub->publish(reset_msg);
    //ros::spinOnce();
    //loop_rate.sleep();
    //std::this_thread::sleep_for(timespan);
  }
  
  
  std::cout << "Calibration Procedure\n-----\n"
            << "Rotate joints so that each slot is aligned with its\n" 
            << "corresponding white mark\nBE SURE TO MOVE AT LEAST ONE JOINT "
            << "BACKWARDS AND FORWARDS A FEW DEGREES!\nIF YOU DO NOT "
            << "THE EDO WILL NOT CALIBRATE CORRECTLY AND WILL NEED TO BE "
            << "RESET!\n";
  while(proceed != 'y'){
    std::cout << "Enter 'y' to enter JogMode and calibrate each joint: ";
    std::cin >> proceed;
  }  
  proceed = '\n';         // Reset char for next prompt
  
  //jog(nh);
  
  while(proceed != 'y'){
    std::cout << "Enter 'y' to send calibration command: ";
    std::cin >> proceed;
  }
  proceed = '\n';  
  calib_msg.joints_mask = 63;

  while(calib_pub->get_subscription_count() == 0){
    loop_rate.sleep();
    std::cout << "checking subscribers to /bridge_jnt_calib... \n";
  }
  
  
  calib_pub->publish(calib_msg);
  //ros::spinOnce();
  // rclcpp::spin(node);
  //loop_rate.sleep();
  //std::this_thread::sleep_for(timespan);


  
}  // calibrate()

bool initialStartup(rclcpp::executors::SingleThreadedExecutor& exec, std::shared_ptr<rclcpp::Node> node){ //rclcpp::Node node
  //ros::Rate loop_rate(100);
  //rclcpp::Rate loop_rate(10000);

  auto stateChecker = std::make_shared<StateChecker>();
  //auto stateChecker2 = std::make_shared<StateChecker>();
  std::chrono::nanoseconds timeout;
  timeout= std::chrono::nanoseconds { 2000000000 };
  exec.add_node(stateChecker);
  char option = 'y';

  do{

      while(!stateChecker->getStateReceived() ){
          exec.spin_once(timeout);

          std::cout << "checking machine state... \n";
          
          }
 
  
  
    

      //int state = check.getState();
      int state = stateChecker->getState();
      switch(state){

        case 0:
          std::cout << "eDO is in state INIT.\nLaunching calibration...\n";
          calibrate(node, false);
          return true;          // OK to continue
      
        case 1:
          std::cout << "eDO is in state NOT_CALIBRATE.\n"
                    << "Launching calibration...\n";
          calibrate(node, false);
          return true;          // OK to continue

        case 2:
          std::cout << "eDO is in state CALIBRATE.\nNo need to calibrate.\n";
          return true;          // OK to continue
        
        case 3:
          std::cout << "eDO is in state MOVE.\nIs the tablet controller in use?\n"
                    << "Fetch new state? Enter y/n: ";
          std::cin >> option;
          if(option != 'y'){
            return false;       // Exit
          }
          break;                // Check state again

        case 4:
          std::cout << "eDO is in state JOG.\nIs the tablet controller in use?\n"
                    << "Fetch new state? Enter y/n: ";
          std::cin >> option;
          if(option != 'y'){
            return false;       // Exit
          }
          break;                // Check state again
     
        case 5:
          std::cout << "eDO is in state MACHINE_ERROR.\nRestart reccommended.\n"
                    << "Terminating edo_manual_ctrl";
          return false;         // Exit
        
        case 6:
          std::cout << "eDO is in state BRAKED.\nRestart reccommended.\n"
                    << "Terminating edo_manual_ctrl";
          return false;

        case 255:
          std::cout << "eDO is in state COMMAND_STATE.\n"
                    << "Fetch new state? Enter y/n: ";
          std::cin >> option;
          if(option != 'y'){
            return false;       // Exit
          }
          break;                // Check state again

        default:
          std::cout << "eDO is in an unknown state.\nRestart recommended.\n"
                    << "Terminating edo_manual_ctrl";
          return false;

      }  // switch(state)
  } while(option == 'y');
  
return false;
}  // initialStartup()


int main(int argc, char **argv){
  
  // Initialize "edo_manual_ctrl" ROS node and NodeHandle for Publishers and
  // Subscribers
 // ros::init(argc, argv, "edo_manual_ctrl");
  //ros::NodeHandle nh;
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("edo_manual_ctrl"); 
  rclcpp::executors::SingleThreadedExecutor exec;
 
 // uncomment this if you want to test calibrate alone
 //calibrate(node, false);

  std::cout << std::fixed;                // Set precision of decimals to 
  std::cout << std::setprecision(2);      // 2 decimal places for output

  
 // calibrate(node->get_node_base_interface(), false);

  // Create/run initial startup to check e.DO state and calibrate if necessary
  // If bad state, exit and return -1
  // COMMENT THIS OUT IF YOU WANT TO AVOID HAVING TO MANUALLY PUBLISH MESSAGES 
  // AND ECHO TOPICS. iF YOU WANT TO TEST THAT THIS IS WORKING, OPEN A DIFFERENT 
  // TERMINAL OR TERMINALS AND USE THE FOLLOWING COMMANDS TO SIMULATE THE SUBSCRIBERS
  // AND PUBLISHERS THAT WOULD BE IN THE REAL EDO ROBOT:
  // TERMINAL1: ros2 topic pub /machine_state edo_core_msgs/msg/MachineState '{current_state: 1, opcode: 4}'
  // TERMINAL2: ros2 topic echo /bridge_init
  // TERMINAL3: ros2 topic echo /bridge_jnt_reset
  // TERMINAL4: ros2 topic echo /bridge_jnt_calib
  // NOTE: IF YOU GET THE ERROR THAT ROS2 CANT FIGURE OUT THE TYPE FOR THE TOPICS, THEN RUN THE COMMANDS 
  // AFTER YOU HAVE STARTED THIS PROGRAM BUT BEFORE YOU TYPE 'Y' FOR ANY OF THE PROMPTS
  if(!initialStartup(exec, node)){
    return -1;
  }
  

 
  
  int choice = 0;

  do {
    std::cout << "1 for jog control\n"
              << "2 for move control\n"
              << "3 to re-calibrate\n"
              << "4 to print eDO data\n"
              << "-1 to exit: ";
    std::cin >> choice;

    switch(choice){

    case 1:
      //jog(nh);
      break;
    
    case 2:
     // move(nh);
      break;
      
    case 3:
     // calibrate(nh, true);
      break;

    case 4:
     // getData(nh);
      break;
 
    } // switch(choice)
  } while(choice != -1);

  return 0;
}  // main() 


