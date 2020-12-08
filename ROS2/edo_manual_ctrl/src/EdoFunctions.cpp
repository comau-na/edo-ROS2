//#include <ros/ros.h>
#include <functional>
#include <memory>
#include "rclcpp/rclcpp.hpp" //ros2

#include "MovementCommandQueue.h"
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

// ===============================================================
// Simply creates a MovementCommand message with values that correllate to a jog message
// ===============================================================
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

edo_core_msgs::msg::MovementCommand createMove(int type, int delay){

  edo_core_msgs::msg::MovementCommand msg;
  // Joint movement to joint point
  if(type == 0){
    msg.move_command = 77;
    msg.move_type = 74;
    msg.ovr = 100;
    msg.delay = delay;
    msg.target.data_type = 74;
    msg.target.joints_mask = 127;//63;
    msg.target.joints_data.resize(6, 0.0);
  }
  // Joint movement to cartesian point
  else if(type == 1){
    msg.move_command = 77;
    msg.move_type = 74;
    msg.ovr = 100;
    msg.delay = delay;
    msg.target.data_type = 88;
    msg.target.joints_mask = 127;//63;
    msg.target.joints_data.resize(10, 0.0);
  }
  // Linear movement to joint point
  else if(type == 10){
    msg.move_command = 77;
    msg.move_type = 76;
    msg.ovr = 100;
    msg.delay = delay;
    msg.target.data_type = 74;
    msg.target.joints_mask = 127;//63;
    msg.target.joints_data.resize(6, 0.0);
  }
  // Linear movement to cartesian point
  else if(type == 11){
    msg.move_command = 77;
    msg.move_type = 76;
    msg.ovr = 100;
    msg.delay = delay;
    msg.target.data_type = 88;
    msg.target.joints_mask = 127;//63;
    msg.target.joints_data.resize(10, 0.0);
  }
  // Reset
  else if(type == -1){
    msg.move_command = 67;
    msg.target.joints_data.clear();
  }
  return msg;
}  // createMove()






// ===============================================================
// This function actually publishes the message
// ===============================================================
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

  // now publish the message
  jog_ctrl_pub->publish(msg);

  loop_rate.sleep();
}  // jogHelper()

// ===============================================================
// This function allows the user to move each joint in real-time
// ===============================================================
void jog(std::shared_ptr<rclcpp::Node> node){

 
  rclcpp::WallRate loop_rate(100);

  // create publisher for bridge_jog topic
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

    // Switch decides which joint to move and which direction
    switch(ch) {

      case 'q':
      case 'Q':
        jogHelper(msg, 1, jog_ctrl_pub, loop_rate, velocity);
        break;

      case 'a':
      case 'A':
        jogHelper(msg, -1, jog_ctrl_pub, loop_rate, velocity);
        break;

      case 'w':
      case 'W':
        jogHelper(msg, 2, jog_ctrl_pub, loop_rate, velocity);
        break;

      case 's':
      case 'S':
        jogHelper(msg, -2, jog_ctrl_pub, loop_rate, velocity);
        break;

      case 'e':
      case 'E':
        jogHelper(msg, 3, jog_ctrl_pub, loop_rate, velocity);
        break;

      case 'd':
      case 'D':
        jogHelper(msg, -3, jog_ctrl_pub, loop_rate, velocity);
        break;

      case 'r':
      case 'R':
        jogHelper(msg, 4, jog_ctrl_pub, loop_rate, velocity);
        break;

      case 'f':
      case 'F':
        jogHelper(msg, -4, jog_ctrl_pub, loop_rate, velocity);
        break;

      case 't':
      case 'T':
        jogHelper(msg, 5, jog_ctrl_pub, loop_rate, velocity);
        break;

      case 'g':
      case 'G':
        jogHelper(msg, -5, jog_ctrl_pub, loop_rate, velocity);
        break;

      case 'y':
      case 'Y':
        jogHelper(msg, 6, jog_ctrl_pub, loop_rate, velocity);
        break;

      case 'h':
      case 'H':
        jogHelper(msg, -6, jog_ctrl_pub, loop_rate, velocity);
        break;

      //Gripper moves
      case 'i':
      case 'I':
        jogHelper(msg, 7, jog_ctrl_pub, loop_rate, velocity);
        break;

      case 'k':
      case 'K':
        jogHelper(msg, -7, jog_ctrl_pub, loop_rate, velocity);
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

// ================================================================
// Calibrate function: This publishes messages to initialize the robot and disengage the brakes
// It uses the jog function to allow the user to manually move each joint to is home location
// ================================================================
void calibrate(std::shared_ptr<rclcpp::Node> node, bool recalib){ 


  // create ros2 publisher
  auto calib_pub = node->create_publisher<edo_core_msgs::msg::JointCalibration>("/bridge_jnt_calib", 1000);
  //rclcpp::executors::SingleThreadedExecutor exec;

  rclcpp::WallRate loop_rate(10);
  edo_core_msgs::msg::JointCalibration calib_msg;
  std::chrono::milliseconds timespan(10000);   // To sleep program for x seconds


  char proceed = '\n';  // Char to allow user to control when commands are sent


  if(!recalib){

    // use this to publish the reset message
    auto reset_pub = node->create_publisher<edo_core_msgs::msg::JointReset>("/bridge_jnt_reset", 10);
    edo_core_msgs::msg::JointReset reset_msg;

    // use this to publish the initialization message
    auto init_pub = node->create_publisher<edo_core_msgs::msg::JointInit>("/bridge_init", 10);
    edo_core_msgs::msg::JointInit init_msg;


    while(proceed != 'y'){
      std::cout << "Enter 'y' to initialize 6-Axis eDO w/o gripper: ";
      std::cin >> proceed;
    }
    proceed = '\n'; // Reset char for next prompt

    // set the values for the init message
    init_msg.mode = 0;
    init_msg.joints_mask = 63;
    init_msg.reduction_factor = 0.0;

    std::cout << "/bridge_init sub count: " << init_pub->get_subscription_count() << "\n";

    // wait until there is a subscriber before you publish the message
    while(init_pub->get_subscription_count() == 0 ){
      loop_rate.sleep();
      std::cout << "checking for Subscribers to /bridge_init... \n";
    }

    // now publish the message
    init_pub->publish(init_msg);
 
    std::this_thread::sleep_for(timespan);      // while e.DO initializes


    while(proceed != 'y'){
      std::cout << "Enter 'y' to disengage brakes: ";
      std::cin >> proceed;
    }
    proceed = '\n'; // Reset char for next prompt

    // initialize the reset message values
    reset_msg.joints_mask = 63;
    reset_msg.disengage_steps = 2000;
    reset_msg.disengage_offset = 3.5;

    // wait until there is a subscriber listening
    while(reset_pub->get_subscription_count() == 0){
      loop_rate.sleep();
      std::cout << "Checking for Subscribers to /bridge_jnt_reset... \n";
    }

    // now publish the message
    reset_pub->publish(reset_msg);

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

  // call the jog function to allow the user to manually calibrate
  jog(node);

  while(proceed != 'y'){
    std::cout << "Enter 'y' to send calibration command: ";
    std::cin >> proceed;
  }
  proceed = '\n';

  // set the value for the calibration message
  calib_msg.joints_mask = 63;

  // wait until there is a subscriber listening
  while(calib_pub->get_subscription_count() == 0 ){
    loop_rate.sleep();
    std::cout << "checking subscribers to /bridge_jnt_calib... \n";
  }


  // now publish the message
  calib_pub->publish(calib_msg);

}  


// ================================================================
// Print e.DO Data function: This function prints the current e.DO data and returns it to the commmand line.
// ================================================================
void getData(rclcpp::executors::SingleThreadedExecutor& exec){

  auto data = std::make_shared<DataDisplay>();  // instantiates a DataDisplay object
  exec.add_node(data); //creates a ROS2 node from the object

  //stay in loop until data has been printed back to the user in the command line
  while(rclcpp::ok() && !(data->getCartesianPrinted() && data->getStatePrinted()&& data->getJointPrinted())){
    exec.spin_some(); //complete queued work in the SingleThreadedExecutor 
    }
    exec.remove_node(data);  //destroys node after data has been printed ot the command line
  }


// ========================================================================
// This function should be called before going to the main menu of this program.
// It checks if the machine is in use by another program or if it has already been calibrated.
// ========================================================================
bool initialStartup(rclcpp::executors::SingleThreadedExecutor& exec, std::shared_ptr<rclcpp::Node> node){ //rclcpp::Node node

  // initialize state checker class
  auto stateChecker = std::make_shared<StateChecker>();
 

  std::chrono::nanoseconds timeout;
  timeout= std::chrono::nanoseconds { 2000000000 };

  // executors allow for more options with spinning nodes
  exec.add_node(stateChecker);
  char option = 'y';

  do{
      // error handling that says do not move forward until the machine state has been received
      while(!stateChecker->getStateReceived() ){
          exec.spin_once(timeout);

          std::cout << "checking machine state... \n";

          }

  // always remove nodes from an executor once you are done using it
 exec.remove_node(node);


    // switch statement that selects the correct option based on machine state
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



void move()
{
  //debug
auto move_ctrl = std::make_shared<MovementCommandQueue>();
 // MovementCommandQueue move_ctrl();

  int anglesOrCartesian, numEntries = 0, delay = 0;   // Vars to save user input

  // Output control information
  std::cout << "Select move type:\n"
            << "0 - joint movement to joint point\n"
            << "1 - joint movement to cartesian point\n"
            << "10 - cartesian movement to joint point\n"
            << "11 - cartesian movement to cartesian point\n";
  std::cin  >> anglesOrCartesian;
  std::cout << "Enter number of entries: ";
  std::cin  >> numEntries;
  std::cout << "Enter delay: ";
  std::cin  >> delay;
  std::cout << "Enter ";
  if(anglesOrCartesian == 0 || anglesOrCartesian == 10){
    std::cout << "joint angles as follows and press enter: J1 J2 J3 J4 J5 J6\n";
  }
  else{
    std::cout << "cartesian coordinates as follows and press enter: "
              << "X Y Z A E R\n";
  }
  std::vector<edo_core_msgs::msg::MovementCommand> pointVec;

  // Read each entry
  for(int i = 0; i < numEntries; ++i){
    edo_core_msgs::msg::MovementCommand msg = createMove(anglesOrCartesian, delay);
    if(anglesOrCartesian == 0 || anglesOrCartesian == 10){
      for(int x = 0; x < 6; ++x){
        scanf("%f", &msg.target.joints_data[x]);
      }
    }
    else{
      scanf("%f", &msg.target.cartesian_data.x);
      scanf("%f", &msg.target.cartesian_data.y);
      scanf("%f", &msg.target.cartesian_data.z);
      scanf("%f", &msg.target.cartesian_data.a);
      scanf("%f", &msg.target.cartesian_data.e);
      scanf("%f", &msg.target.cartesian_data.r);
    }
    pointVec.push_back(msg);        // Store all waypoints in vector
                                    // to be executed
  }

  int numLoops = 0;
  while (numLoops < 1){
    std::cout << "Enter number of loops: ";
    std::cin >> numLoops;
    if (numLoops < 1){
      std::cout << "Number of loops must be at least 1.\n";
    }
  }
  // Push waypoints from vector to queue system in MoveCommandQueue class
  for(int i = 0; i < numLoops; ++i){
    std::vector<edo_core_msgs::msg::MovementCommand>::iterator it = pointVec.begin();
    while(it != pointVec.end()){
      move_ctrl->pushMoveCommand(*it);
      it++;
    }
  }
  // While loop exits when move queue is done running
  while(rclcpp::ok() && move_ctrl->stillRunning()){
    rclcpp::spin_some(move_ctrl);
  }

}  // move()
