#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include "std_msgs/Bool.h"
#include <string>

ros::Publisher pose_pub;
ros::Publisher gripper_pub;
geometry_msgs::Pose pose;
std_msgs::Bool gripper_open;
std_msgs::Bool gripper_close;

int setPose = 0;

float px = 0.33272;
float py = 0.028956;
float pz = 0.28955;
float ox =  0.00014;
float oy = 0.99968;
float oz =  -0.00010;
float ow =  -0.025302;  

void handPoseMsgCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("The Classification of the hand is : [%s]", msg->data.c_str());
  std::string gesture = msg->data.c_str();
    
	
	
	
	if(gesture == "no hand")
		gripper_pub.publish(gripper_close);
  
  // Moves the e.DO in the positive direction of the y-axis (looking at the robot from the front it will go right)
  while(gesture == "pan") {
	 pose.position.x =  px;
	 pose.position.y = py + 0.01; 
	 pose.position.z = pz;
	 pose.orientation.x = ox;
	 pose.orientation.y = oy;
	 pose.orientation.z = oz;
	 pose.orientation.w = ow; 
	 pose_pub.publish(pose);
	  
  }
	// Moves the e.DO in the negative direction of the y-axis (looking at the robot from the front it will go left)
  while (gesture == "peace"){
	 pose.position.x =  px;
	 pose.position.y = py - 0.01; 
	 pose.position.z = pz;
	 pose.orientation.x = ox;
	 pose.orientation.y = oy;
	 pose.orientation.z = oz;
	 pose.orientation.w = ow; 
	 pose_pub.publish(pose);
 }
  
    //Opens the gripper, lowers the e.DO robot on the Z-axis over a block, closes the gripper and raises it. 
  while(gesture == "fist"){
	  // opens the gripper
	  gripper_pub.publish(gripper_open);
	  
	  //lowers the arm
	  pz = 0.1443;
	  
	 pose.position.x =  px;
	 pose.position.y = py; 
	 pose.position.z = pz;
	 pose.orientation.x = ox;
	 pose.orientation.y = oy;
	 pose.orientation.z = oz;
	 pose.orientation.w = ow; 
	 pose_pub.publish(pose);
	 
	  //closes the gripper
	 gripper_pub.publish(gripper_close);
	 
	  //raises the arm
	pz = 0.28955;
 
	 pose.position.x =  px;
	 pose.position.y = py; 
	 pose.position.z = pz;
	 pose.orientation.x = ox;
	 pose.orientation.y = oy;
	 pose.orientation.z = oz;
	 pose.orientation.w = ow; 
	 pose_pub.publish(pose);
	 pose_pub.publish(pose);

  }
//Moves the e.DO in the positive direction of the X- axis for the arm to reach the buckets
while (gesture == "ok") {
	pose.position.x =  px +  0.005;
	 pose.position.y = py ; 
	 pose.position.z = pz;
	 pose.orientation.x = ox;
	 pose.orientation.y = oy;
	 pose.orientation.z = oz;
	 pose.orientation.w = ow; 
	  pose_pub.publish(pose);
	  
}

while (gesture == "stop") {
	  // opens the gripper
	 gripper_pub.publish(gripper_open);

	  //resets the position
	  
	 px = 0.33272;
	 py = 0.028956;
     pz = 0.28955;
     ox =  0.00014;
     oy = 0.99968;
     oz =  -0.00010;
     ow =  -0.025302;  
	  
	  
	 pose.position.x =  px;
	 pose.position.y = py; 
	 pose.position.z = pz;
	 pose.orientation.x = ox;
	 pose.orientation.y = oy;
	 pose.orientation.z = oz;
	 pose.orientation.w = ow; 
	 pose_pub.publish(pose);
}

  
  
    ros::shutdown();
}

int main(int argc, char **argv)
{
	
  ros::init(argc, argv, "gesture_listener");
  ros::NodeHandle n;

  gripper_open.data = true;
  gripper_close.data = false;
  pose_pub = n.advertise<geometry_msgs::Pose>("/edoMove", 100);
  gripper_pub =n.advertise<std_msgs::Bool>("/open_gripper", 1000);

  ros::Subscriber sub = n.subscribe("gesture_class", 100, handPoseMsgCallback);
  
  ros::spinOnce();

  return 0;
}
