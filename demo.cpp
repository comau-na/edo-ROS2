#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <string>

#include "ros/ros.h"
#include "std_msgs/String.h"

std::string gesture;

void handPoseMsgCallback(const std_msgs::String::ConstPtr& msg)
{
  gesture = msg->data;
  ROS_INFO("The Position of the hand is : [%s]", msg->data.c_str());
	
}

int main(int argc, char** argv)
{	
   ros::init(argc, argv, "gesture_listener");
   ros::NodeHandle n;
   ros::Subscriber sub = n.subscribe("gesture_class", 100, handPoseMsgCallback);
   //ros::spin();

	std::cout << gesture << std::endl;

   int gestureID;
   if(gesture == "pan")
	{
		gestureID = 1;
	}
	else if(gesture == "stop")
	{
		gestureID = 2;
	}
	else if(gesture == "peace")
	{
		gestureID = 3;
	}
	else if(gesture == "fist")
	{
		gestureID = 4;
	}
	else if(gesture == "ok")
	{
		gestureID = 5;
	}
	else
	{
		gestureID = 0;
	}

   std::cout << "Joe" << std::endl;
   ros::init(argc, argv, "demo");
   ros::NodeHandle node_handle;
   ros::AsyncSpinner spinner(1);
   spinner.start();

  //initializes the planning group
  static const std::string PLANNING_GROUP = "edo";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  const robot_state::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);



// // Get the classification for the gesture then set it to the correct gestureID
// 0 = no hand
// 1 = Pan
// 2 = Stop
// 3 = Peace
// 4 = Fist
// 5 = ok
  
  //move_group.setRandomTarget();

geometry_msgs::Pose target_pose;
switch(gestureID) {
	
	// Pan Pose
				case 1: 
							target_pose.position.x = 0.389834640181;
							target_pose.position.y = -5.13314643673e-05;
							target_pose.position.z = 1.48997718215;
							target_pose.orientation.x = 4.21981722855e-05;
							target_pose.orientation.y = 0.950104675016;
							target_pose.orientation.z = -3.84753687063e-05;
							target_pose.orientation.w = 0.311931247635;
							break;

	// Stop (candle) pose
				case 2: 
							target_pose.position.x = 9.88121415998e-05;
							target_pose.position.y = 5.92231270015e-09;
							target_pose.position.z = 2.12999999345;
							target_pose.orientation.x = -9.51910637009e-10;
							target_pose.orientation.y = 5.96880595398e-05;
							target_pose.orientation.z = 5.28870033485e-05;
							target_pose.orientation.w = 0.99999999682;
							break;
	
	// Peace bend middle joint at 45 degree angle
				case 3: 
							target_pose.position.x = 0.397283811145;
							target_pose.position.y = -3.98077073662e-05;
							target_pose.position.z = 1.96567767916;
							target_pose.orientation.x = 1.84874275807e-06;
							target_pose.orientation.y = 0.382173407818;
							target_pose.orientation.z =-8.81235549789e-05;
							target_pose.orientation.w = 0.924090622497;
							break;
// fist - bend top joint at 90 degree angle
				case 4: 
							target_pose.position.x = 0.294512422741;
							target_pose.position.y = -1.18905328599e-05;
							target_pose.position.z = 1.83634513545;
							target_pose.orientation.x = -1.52722533285e-05;
							target_pose.orientation.y = 0.706091448757;
							target_pose.orientation.z = -4.3907904711e-05;
							target_pose.orientation.w = 0.708120656267;
							break;
//ok - bend bottom joint at 90 degree angle
				case 5: 
							target_pose.position.x = 0.77299087626;
							target_pose.position.y = -0.000307060946931;
							target_pose.position.z = 1.35325812948;
							target_pose.orientation.x = 0.000137034943648;
							target_pose.orientation.y = 0.708839058867;
							target_pose.orientation.z =  -0.000143836004292;
							target_pose.orientation.w = 0.705370221342 ;
							break;

				default:
							target_pose.position.x = 9.88121415998e-05;
							target_pose.position.y = 5.92231270015e-09;
							target_pose.position.z = 2.12999999345;
							target_pose.orientation.x = -9.51910637009e-10;
							target_pose.orientation.y = 5.96880595398e-05;
							target_pose.orientation.z = 5.28870033485e-05;
							target_pose.orientation.w = 0.99999999682;
							break;
}

  move_group.setPoseTarget(target_pose);
  ROS_INFO_NAMED("move_to_pose", "Setting the target position to x=%g, y=%g, z=%g",target_pose.position.x, target_pose.position.y, target_pose.position.z);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);  
  move_group.move();
  ros::shutdown();
  return 0;
}
