#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char** argv)
{
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

  
  //move_group.setRandomTarget();

  //set pose 
  geometry_msgs::Pose target_pose;
  target_pose.position.x = -0.6847;
  target_pose.position.y = 0.001898;
  target_pose.position.z = 1.04094;
  target_pose.orientation.x = -0.34125;
  target_pose.orientation.y = -0.82208;
  target_pose.orientation.z = 0.17362;
  target_pose.orientation.w = 0.42132;
  move_group.setPoseTarget(target_pose);
  ROS_INFO_NAMED("move_to_pose", "Setting the target position to x=%g, y=%g, z=%g",target_pose.position.x, target_pose.position.y, target_pose.position.z);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);  
  move_group.move();
  ros::shutdown();
  return 0;
}
