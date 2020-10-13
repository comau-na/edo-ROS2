/** @file Relay.cpp
 *  @brief Class reads std_msgs::Int8MultiArray published by e.DO simulation
 *  and translates them and sends them to e.DO via MovementCommandQueue class
 *  @author Jack Shelata and Ashwini Magar
 *  @date August 2, 2018
 */

#include "Relay.h"


/***************************************************************
**                Function(s) Definition
****************************************************************/

/** @brief Construct Relay object, initialize ROS node handle and use it to
 *  create subscriber to "/relay" topic. Create new MovementCommandQueue object
 *  and initialize the quick option
 *  @param nh_in - ROS node handle by reference
 *  @param quick_in - If true send only contact and reset points to e.DO
 *  @return Relay object
 *  @exception None
 **/
Relay::Relay(ros::NodeHandle& nh_in, bool quick_in){
  nh = nh_in;
  relaySub = nh_in.subscribe("/relay", 1, &Relay::relayCallback, this);
  move_ctrl = new MovementCommandQueue(nh);
  quick = quick_in;
}  // Relay::Relay()

/** @brief Destructor properly destroys Relay object
 *  @param None
 *  @return void
 *  @exception None
 **/
Relay::~Relay(){
  // Delete move_ctrl object since new was called
  delete move_ctrl;
}  // Relay::~Relay()

/** @brief Callback function runs then new win sequence is received on "/relay"
 *  topic
 *  @param msg - Int8MultiArray message from "/relay"
 *  @return void
 *  @exception None
 **/
void Relay::relayCallback(const std_msgs::Int8MultiArray& msg){
  // If there is no move sequence running on e.DO
  if(!move_ctrl->stillRunning()){
    ros::Rate loop_rate(100);

    // Initialize empty MovementCommand message and set default values
    edo_core_msgs::MovementCommand empty;
    empty.move_command = 77;                  // Move message
    empty.move_type = 74;                     // Joint movement
    empty.ovr = 100;
    empty.delay = 255;                        // Fly
    empty.target.data_type = 74;              // Joint angles
    empty.target.joints_mask = 127;//Joint mask should set to 127 for gripper and 63 for without gripper

    empty.target.joints_data.resize(7, 0.0);  // Resize and intialize angles

    // Create vector to hold all message points in sequence
    // Resize to incoming array size/6 (6-axis) and initialize all to empty
    std::vector<edo_core_msgs::MovementCommand> pointVec;
    pointVec.resize(msg.data.size()/6, empty); 
    
    // Loop through all points
    for(int i = 0; i < pointVec.size() ; ++i){
      // Loop through each angle in one point
      for(int x = 0; x < 6; ++x){
        pointVec[i].target.joints_data[x] = msg.data[i*6+x];
        }
    }

     // open gripper to max(80mm)for the last 5 points on the end
    for(int j=2;j<7;j++){
    pointVec[pointVec.size()-j].target.joints_data[6] = 80;
    }

    // Create vector to hold every other message to shorten sequence
    // Contact point and reset point must be kept
    std::vector<edo_core_msgs::MovementCommand> pointVecShort;
    
    // Loop all but last two points and add every other point
    for(int i = 0; i < pointVec.size()-2; i+=2){
      pointVecShort.push_back(pointVec[i]);
    }
       
    // Add the last two points on the end
    pointVecShort.push_back(pointVec[pointVec.size()-2]);//2-contact point
    
      //close the gripper
    pointVec[pointVec.size()-2].target.joints_data[6] = 50;  
    pointVecShort.push_back(pointVec[pointVec.size()-2]);
    
      //pick cylinder up at new position 20
    pointVec[pointVec.size()-2].target.joints_data[1] = 20;
    pointVec[pointVec.size()-2].target.joints_data[6] = 50;  
    pointVecShort.push_back(pointVec[pointVec.size()-2]);

     // release cylinder at position 20
    pointVec[pointVec.size()-2].target.joints_data[1] = 20;
    pointVec[pointVec.size()-2].target.joints_data[6] = 77;  
    pointVecShort.push_back(pointVec[pointVec.size()-2]);

    pointVec[pointVec.size()-1].target.joints_data[6] = 1;//close gipper at top
    pointVecShort.push_back(pointVec[pointVec.size()-1]);//1-reset point

        
    //wait time for gripper actions 
    pointVecShort[pointVecShort.size()-5].delay = 2; // Pause 2sec on contact
    pointVecShort[pointVecShort.size()-4].delay = 1;
    pointVecShort[pointVecShort.size()-3].delay = 1; //pause on hold
    pointVecShort[pointVecShort.size()-2].delay = 1; //pause on release
    

     // Check that the last point is the reset positon and add reset position
    // if the last position is not the reset positon
    // Should match reset postion or simulation in degrees 
    if(pointVecShort[pointVecShort.size()-1].target.joints_data[1] != 0 ||
      pointVecShort[pointVecShort.size()-1].target.joints_data[2] != 14){

      pointVec.push_back(empty);
      pointVecShort[pointVecShort.size()-1].target.joints_data[1] = 0;
      pointVecShort[pointVecShort.size()-1].target.joints_data[2] = 14;
    }

    // If quick is true send only the contact point and the reset position
    // Else send all positons in pointVecShort
    if(quick){
        move_ctrl->pushMoveCommand(pointVecShort[pointVecShort.size()-2]);
        move_ctrl->pushMoveCommand(pointVecShort[pointVecShort.size()-1]);
    }
    else{
      for(int i = 0; i < pointVecShort.size(); i++){
        move_ctrl->pushMoveCommand(pointVecShort[i]);
      }
    } 
    std::cout << "MovementCommandQueue filled\n";
  }
  else{
    std::cout << "Busy...\n";
  }  
}  // Relay::relayCallback()

// Getter member funciton to check if the last move sequence is still running on
// e.DO
bool Relay::getStillRunning(){
  return move_ctrl->stillRunning();
}  // Relay::getStillRunning()
