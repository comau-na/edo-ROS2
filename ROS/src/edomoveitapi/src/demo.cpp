#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

geometry_msgs::Pose target_pose;
static const std::string PLANNING_GROUP = "edo";

void edomoveCallback(const geometry_msgs::Pose::ConstPtr& msg){
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  target_pose = *msg.get();
  std::cout << "Pose published" << std::endl;
  move_group.setPoseTarget(target_pose);
  ROS_INFO_NAMED("move_to_pose", "Setting the target position to x=%g, y=%g, z=%g",target_pose.position.x, target_pose.position.y, target_pose.position.z);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);  
  move_group.move();
  std::cout << "Ready to listen" << std::endl;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "demo");
  ros::AsyncSpinner spinner(2);

  spinner.start();
  geometry_msgs::
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  const robot_state::JointModelGroup* joint_model_group =
    move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  spinner.stop();
  spinner.start();
  std::cout << "Ready to listen" << std::endl;
  
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("edoMove", 1000, edomoveCallback);
  
  ros::waitForShutdown();

  return 0;
}
