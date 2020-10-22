//#include <ros/ros.h>
#include <functional>
#include <memory>
#include "rclcpp/rclcpp.hpp" //ros2

//#include "MovementCommandQueue.h"
#include "DataDisplay.h"
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
#include <ncurses.h>  // Used in the jog function to allow for keystrokes to
                      // be captured without an enter press

using namespace std::chrono_literals;

/** @brief Function manages jog command creation. Fills in Jog type values
 *  and returns a message to be filled with velocity values on scale from
 *  -1.0 to 1.0.
 *  @param None
 *  @return MovementCommand - Message defined in edo_core_msgs ROS package
 *  @exception None
 */  
edo_core_msgs::msg::MovementCommand createJog(){
  edo_core_msgs::msg::MovementCommand msg;
  msg.move_command = 74;
  msg.move_type = 74;
  msg.ovr = 100;
  msg.target.data_type = 74;
  msg.target.joints_mask = 127;
  msg.target.joints_data.resize(10, 0.0);
  return msg;

}  // createJog()

/** @brief Function to carry out each button press jog command
 *  @param msg - existing MovementCommand to be edited and sent
 *  @param joint_number - int number of the joint to be moved (1-6)
 *  @param jog_ctrl_pub - ROS Publisher to publish the jog message
 *  "/bridge_jog" ROS topic
 *  @param loop_rate - ROS Rate message for set sleep time
 *  @param velocity - double velocity value for jog
 *  @return void
 *  @exception None
 */  
void jogHelper(edo_core_msgs::msg::MovementCommand& msg, int joint_number,
    std::shared_ptr<rclcpp::Publisher<edo_core_msgs::msg::MovementCommand_<std::allocator<void> >, std::allocator<void> > > jog_ctrl_pub, rclcpp::WallRate& loop_rate, double velocity){ 
  msg.target.joints_data.clear();     
  msg.target.joints_data.resize(10,0.0);
  if(joint_number > 0){
    std::cout << "\rJoint " << joint_number << " + " << velocity << std::flush;
    msg.target.joints_data[joint_number - 1] = velocity;
  }
  else{
    std::cout << "\rJoint " << -1 * joint_number << " - "
              << velocity << std::flush;
    msg.target.joints_data[(-1 * joint_number) - 1] = -1 * velocity;
  }
  jog_ctrl_pub->publish(msg);
 // rclcpp::spin_once();
  loop_rate.sleep();
}  // jogHelper()

/** @brief Function manages sending jog commands using ncurses library for
 *  push-button key capturing
 *  @param nh - ROS NodeHangle for creating jog publisher
 *  @return void
 *  @exception None
 */  
void jog(std::shared_ptr<rclcpp::Node> node){

  //ros::Rate loop_rate(100);
  rclcpp::WallRate loop_rate(100);
  //ros::Publisher jog_ctrl_pub =  nh.advertise<edo_core_msgs::MovementCommand>("/bridge_jog",10);
  auto jog_ctrl_pub = node->create_publisher<edo_core_msgs::msg::MovementCommand>("/bridge_jog", 10);
  edo_core_msgs::msg::MovementCommand msg = createJog();
  char ch = '\n';     // Char to hold keypress value (Ncurses)

  // Output control information
  std::cout << "-----\nJog Controls (Press and Hold):\n"
      << "Joint 1 +/-: 'q'/'a'\n"
      << "Joint 2 +/-: 'w'/'s'\n" 
      << "Joint 3 +/-: 'e'/'d'\n"
      << "Joint 4 +/-: 'r'/'f'\n" 
      << "Joint 5 +/-: 't'/'g'\n"
      << "Joint 6 +/-: 'y'/'h'\n"
      << "gripper open/close-: 'i'/'k'\n"
      << "Set Velocity +/-: 'u'/'j'\n"
      << "Exit: 'x'\n-----\n"
      << "NOTE: VELOCITY STARTS AT 100%\n";
  while(ch != 'y'){
    std::cout << "Enter 'y' to continue: ";
    std::cin >> ch;
  }
  ch = '\n';
  bool last = false;
  double velocity = 1.0;
  
  
  doupdate();     // Ncurses function to reset window after endwin() has been
             // called
  initscr();      // Ncurses function initializes key capture
  timeout(0);     // Ncurses function set to 0 forces getch() to return
                  // ERR when no key is pressed instead of waiting for key
  curs_set(0);    // Ncurses makes the cursor invisible
  noecho();       // Ncurses function hides pressed keys
 
   do {
    ch = getch(); // Ncurses function returns char of key pressed
                  // returns ERR when no key press
    //std::cin >> ch;
    // Switch decides which joint to move and which direction
    switch(ch) {

      case 'q':
      case 'Q':
        jogHelper(msg, 1, jog_ctrl_pub, loop_rate, velocity);
        last = true;
        break;
          
      case 'a':
      case 'A':
        jogHelper(msg, -1, jog_ctrl_pub, loop_rate, velocity);
        last = true;
        break;
            
      case 'w':
      case 'W':
        jogHelper(msg, 2, jog_ctrl_pub, loop_rate, velocity);
        last = true;
        break;
          
      case 's':
      case 'S':
        jogHelper(msg, -2, jog_ctrl_pub, loop_rate, velocity);
        last = true;
        break;    
        
      case 'e':
      case 'E':
        jogHelper(msg, 3, jog_ctrl_pub, loop_rate, velocity);
        last = true;
        break;
          
      case 'd':
      case 'D':
        jogHelper(msg, -3, jog_ctrl_pub, loop_rate, velocity);
        last = true;
        break;    
            
      case 'r':
      case 'R':
        jogHelper(msg, 4, jog_ctrl_pub, loop_rate, velocity);
        last = true;
        break;
          
      case 'f':
      case 'F':
        jogHelper(msg, -4, jog_ctrl_pub, loop_rate, velocity);
        last = true;
        break;
        
      case 't':
      case 'T':
        jogHelper(msg, 5, jog_ctrl_pub, loop_rate, velocity);
        last = true;
        break;
          
      case 'g':
      case 'G':
        jogHelper(msg, -5, jog_ctrl_pub, loop_rate, velocity);
        last = true;
        break;    
  
      case 'y':
      case 'Y':
        jogHelper(msg, 6, jog_ctrl_pub, loop_rate, velocity);
        last = true;
        break;
          
      case 'h':
      case 'H':
        jogHelper(msg, -6, jog_ctrl_pub, loop_rate, velocity);
        last = true;
        break;

      //Gripper moves
      case 'i':
      case 'I':
        jogHelper(msg, 7, jog_ctrl_pub, loop_rate, velocity);
        last = true;
        break;
          
      case 'k':
      case 'K':
        jogHelper(msg, -7, jog_ctrl_pub, loop_rate, velocity);
        last = true;
        break;
      
      case 'u':
      case 'U':
        if(velocity < 1.0){
          velocity += 0.05;
        }
        std::cout << "\rVelocity: " << velocity << std::flush;
        break;
      
      case 'j':
      case 'J':
        if(velocity > 0.05){
          velocity -= 0.05;
        }
        std::cout << "\rVelocity: " << velocity << std::flush;
        break;
      
      
    }  // switch(choice)
  } while(ch != 'X' && ch != 'x');
  endwin(); // Ends ncurses window

}  // jog()

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

    std::cout << "/bridge_init sub count: " << init_pub->get_subscription_count() << "\n";
    
    while(init_pub->get_subscription_count() == 0 ){
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
  
  jog(node);
  
  while(proceed != 'y'){
    std::cout << "Enter 'y' to send calibration command: ";
    std::cin >> proceed;
  }
  proceed = '\n';  
  calib_msg.joints_mask = 63;

  while(calib_pub->get_subscription_count() == 0 ){
    loop_rate.sleep();
    std::cout << "checking subscribers to /bridge_jnt_calib... \n";
  }
  
  
  calib_pub->publish(calib_msg);
  //ros::spinOnce();
  // rclcpp::spin(node);
  //loop_rate.sleep();
  //std::this_thread::sleep_for(timespan);


  
}  // calibrate()

void getData(std::shared_ptr<rclcpp::Node> node){

  auto dataDisplay = std::make_shared<DataDisplay>(node); 

  DataDisplay data(node);
  while(rclcpp::ok() && !(data.getCartesianPrinted() &&
        data.getStatePrinted() && data.getJointPrinted())){
    //rclcpp::spin_once(node);
    //std::cout << data.printState() << "\n";
    }

  }

bool initialStartup(rclcpp::executors::SingleThreadedExecutor& exec, std::shared_ptr<rclcpp::Node> node){ //rclcpp::Node node
  //ros::Rate loop_rate(100);
  //rclcpp::Rate loop_rate(10000);

  auto stateChecker = std::make_shared<StateChecker>(node);
  //auto stateChecker2 = std::make_shared<StateChecker>();
  std::chrono::nanoseconds timeout;
  timeout= std::chrono::nanoseconds { 2000000000 };
  exec.add_node(node);
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

