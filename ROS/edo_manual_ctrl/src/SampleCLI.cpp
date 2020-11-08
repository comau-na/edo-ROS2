#include "EdoFunctions.cpp"

int main(int argc, char **argv){
  
  // Initialize "edo_manual_ctrl" ROS node and NodeHandle for Publishers and
  // Subscribers
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("edo_manual_ctrl"); 
  rclcpp::executors::SingleThreadedExecutor exec;
 
 // uncomment this if you want to test calibrate alone
 //calibrate(node, false);

  std::cout << std::fixed;                // Set precision of decimals to 
  std::cout << std::setprecision(2);      // 2 decimal places for output

  

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
  
 // if(!initialStartup(exec, node)){
  //  return -1;
  //}
  
  
  
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
      jog(node);
      break;
    
    case 2:
     move(node);
      break;
      
    case 3:
     calibrate(node, true);
      break;

    case 4:
     getData(exec, node);
      break;
 
    } // switch(choice)
  } while(choice != -1);

  return 0;
}  // main() 


