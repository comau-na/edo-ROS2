

/***************************************************************
**                Function(s) Definition
****************************************************************/


/** @file DataDisplay.cpp
 *  @brief Class definiton for DataDisplay - converted for RO2
 *  @date October 8, 2020
 *  @author Seth Buchinger - based on Jack Shelata ROS1 class

*/

#include "DataDisplay.h"

//using std::placeholders::_1;

/**
DataDisplay::DataDisplay(ros::NodeHandle& nh_in){
  nh = nh_in;
  cartesian_pose_sub = nh.subscribe("/cartesian_pose", 10,
      &DataDisplay::printPoseData, this);
  machine_state_sub = nh.subscribe("/machine_state", 10,
      &DataDisplay::printState, this);
  joint_pose_sub = nh.subscribe("/machine_algo_jnt_state", 10,
      &DataDisplay::printJointPose, this);
  cartesianPrinted = false;
  statePrinted = false;
  jointPrinted = false;
}  // DataDisplay::DataDisplay()


*/

DataDisplay::DataDisplay() : Node("data_display")
{

  cartesian_pose_sub_ =
  this->create_subscription<edo_core_msgs::msg::CartesianPose>(
  "/cartesian_pose", 10, std::bind(&DataDisplay::printPoseData, this,_1));
  machine_state_sub_ =
  this->create_subscription<edo_core_msgs::msg::MachineState>(
  "/machine_state", 10, std::bind(&DataDisplay::printState, this,_1));
  joint_pose_sub_ =
  this->create_subscription<edo_core_msgs::msg::JointStateArray>(
  "/machine_algo_jnt_state", 10, std::bind(&DataDisplay::printJointPose,
  this,_1)); 
  cartesianPrinted = false; 
  statePrinted = false; 
  jointPrinted =false;

}

/** @brief Callback function to print CartesianPose message
 *  @param pose - CartesianPose message type from "/cartesian_pose" ROS topic
 *  @return void
 *  @exception None
 */
void DataDisplay::printPoseData(const edo_core_msgs::msg::CartesianPose::SharedPtr pose){
  if(!cartesianPrinted){
    std::cout << "CartesianPose/x: " << pose->x << ", CartesianPose/y: " 
      << pose->y << ", CartesianPose/z: " << pose->z <<"\n";
    cartesianPrinted = true;
  }
}  // DataDisplay::printPoseData()

/** @brief Callback function to print MachineState message
 *  @param pose - MachineState message type from "/machine_state" ROS topic
 *  @return void
 *  @exception None
 */
void DataDisplay::printState(const edo_core_msgs::msg::MachineState::SharedPtr state){
  if(!statePrinted){
    std::cout << "MachineState/current_state: " << unsigned(state->current_state) << "\n";
    std::cout << "MachineState/opcode: " << unsigned(state->opcode) << "\n";
    statePrinted = true;
  }
}  // DataDisplay::printState()

/** @brief Callback function to print JointStateArray message
 *  @param pose - MachineState message type from "/machine_algo_jnt_state"
 *  ROS topic
 *  @return void
 *  @exception None
 */
void DataDisplay::printJointPose(const edo_core_msgs::msg::JointStateArray::SharedPtr pose){
  if(!jointPrinted){
     std::cout << "JointStateArray/joints_mask: " << pose->joints_mask << "\n";
   for(edo_core_msgs::msg::JointState joint : pose->joints)
    {
      std::cout << "edo_core_msgs/JointState[] joints: " << joint.position << "\n";
    }


    jointPrinted = true;
 }
}  // DataDisplay::printJointPose()

/** @brief Getter member function to tell whether cartesian data has been
 *  printed
 *  @param None 
 *  @return bool - Value of cartesianPrinted (true if data was printed)
 *  @exception None
 */

bool DataDisplay::getCartesianPrinted(){
  return cartesianPrinted;
}  // DataDisplay::getCartesianPrinted()

/** @brief Getter member function to tell whether State data has been
 *  printed
 *  @param None 
 *  @return bool - Value of statePrinted (true if data was printed)
 *  @exception None
*/
bool DataDisplay::getStatePrinted(){
  return statePrinted;
}  // DataDisplay::getStatePrinted()

/** @brief Getter member function to tell whether Joint data has been
 *  printed
 *  @param None 
 *  @return bool - Value of jointPrinted (true if data was printed)
 *  @exception None
*/
bool DataDisplay::getJointPrinted(){
  return jointPrinted;
}  // DataDisplay::getJointPrinted()
 