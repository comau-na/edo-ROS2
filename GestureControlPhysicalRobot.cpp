#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Pose.h>
#include <string>

ros::Publisher pose_pub;
geometry_msgs::Pose pose;


void handPoseMsgCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("The Position of the hand is : [%s]", msg->data.c_str());
  std::string gesture = msg->data.c_str();
  
  
  //e.DO Robot will move to the starting position first 
	pose.position.x =  0.33272;
    pose.position.y = 0.028956;
    pose.position.z = 0.28955;
    pose.orientation.x = 0.00014;
    pose.orientation.y = 0.99968;
    pose.orientation.z = -0.00010;
    pose.orientation.w = -0.025302;  
	pose_pub.publish(pose);
  
  //The gesture classification "no hand" will not be shown here. No hand = no movement.
  
  // Moves the e.DO in the positive direction of the y-axis (looking at the robot from the front it will go right)
  while(gesture == "pan") {
	  pose.position.y += 0.01;
	  pose_pub.publish(pose);
	  
  }
	// Moves the e.DO in the negative direction of the y-axis (looking at the robot from the front it will go left)
  while (gesture == "peace"){
	  pose.position.y -= 0.01;
	  pose_pub.publish(pose); 
 }
  
    //Opens the gripper, lowers the e.DO robot on the Z-axis over a block, closes the gripper and raises it. 
  while(gesture == "fist"){
	  // opens the gripper
	  
	  //lowers the arm
	  pose.position.z = 0.1443;
	  pose_pub.publish(pose);
	  
	  //closes the gripper
	  
	  
	  //raises the arm
	  pose.position.z =  0.33272;
	  pose_pub.publish(pose);

  }
//Moves the e.DO in the positive direction of the X- axis for the arm to reach the buckets
while (gesture == "ok") {
	  pose.position.x += 0.005;
	  pose_pub.publish(pose);
	  
}

while (gesture == "stop") {
	  // opens the gripper
	  
	  //resets the position
	  pose.position.x =  0.33272;
	  pose.position.y = 0.028956;
      pose.position.z = 0.28955;
      pose.orientation.x = 0.00014;
      pose.orientation.y = 0.99968;
      pose.orientation.z = -0.00010;
      pose.orientation.w = -0.025302;  
	  pose_pub.publish(pose);
}

  
    ros::shutdown();
}

int main(int argc, char **argv)
{
	
	 //e.DO Robot will move to the starting position first 
		pose.position.x =  0.33272;
		pose.position.y = 0.028956;
		pose.position.z = 0.28955;
		pose.orientation.x = 0.00014;
		pose.orientation.y = 0.99968;
		pose.orientation.z = -0.00010;
		pose.orientation.w = -0.025302;  
		pose_pub.publish(pose);
	
  ros::init(argc, argv, "gesture_listener");
  ros::NodeHandle n;

  pose_pub = n.advertise<geometry_msgs::Pose>("/edoMove", 100);

  ros::Subscriber sub = n.subscribe("gesture_class", 100, handPoseMsgCallback);
  
  ros::spin();

  return 0;
}
