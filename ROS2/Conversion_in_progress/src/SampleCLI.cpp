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


bool initialStartup(rclcpp::executors::SingleThreadedExecutor exec){ //rclcpp::Node node
  //ros::Rate loop_rate(100);
  rclcpp::Rate loop_rate(10);
  return 1;
  /*
  StateChecker check(node);
  char option = 'y';
  do{ 
    while(!check.getStateReceived()){
      ros::spinOnce();
      loop_rate.sleep();
    }
    int state = check.getState();
    switch(state){

      case 0:
        std::cout << "eDO is in state INIT.\nLaunching calibration...\n";
        //calibrate(nh, false);
        return true;          // OK to continue
    
      case 1:
        std::cout << "eDO is in state NOT_CALIBRATE.\n"
                  << "Launching calibration...\n";
        //calibrate(nh, false);
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
  */

}  // initialStartup()


int main(int argc, char **argv){
  
  // Initialize "edo_manual_ctrl" ROS node and NodeHandle for Publishers and
  // Subscribers
 // ros::init(argc, argv, "edo_manual_ctrl");
  //ros::NodeHandle nh;
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("edo_manual_ctrl"); 
  rclcpp::executors::SingleThreadedExecutor exec;


  std::cout << std::fixed;                // Set precision of decimals to 
  std::cout << std::setprecision(2);      // 2 decimal places for output

  
  // Create/run initial startup to check e.DO state and calibrate if necessary
  // If bad state, exit and return -1
  /*if(!initialStartup(*node)){
    return -1;
  }
  */

 auto subscriber_node = std::make_shared<StateChecker>();
  
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


