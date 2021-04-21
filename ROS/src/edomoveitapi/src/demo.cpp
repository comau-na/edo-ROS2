#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <tf2/LinearMath/Quaternion.h>
#include "std_msgs/Bool.h" 
#include "std_msgs/String.h" 
#include "std_msgs/Float32.h" 
#include <vector>

geometry_msgs::Pose target_pose;
static const std::string PLANNING_GROUP = "edo";
ros::Publisher pub;
double positionTolerance, orientationTolerance;

//Callback for movement pose subscriber
void edomoveCallback(const geometry_msgs::Pose::ConstPtr& msg){
  std::vector<geometry_msgs::Pose> waypoints;
  target_pose = *msg.get();
  float wOrientation = target_pose.orientation.w;

  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  // move_group.setPlannerId("LBTRRTkConfigDefault");
  move_group.setPlanningTime(4.0);

  if(wOrientation == 69.0){
    std::cout << "Setting orientation to bucket orientation" << std::endl;
    move_group.setGoalOrientationTolerance(orientationTolerance * 4);
  }else {
    move_group.setGoalOrientationTolerance(orientationTolerance);
  }
  move_group.setGoalPositionTolerance(positionTolerance);

  //If orientation W = 62 or 69, convert Euler angles to Quaternion and move to pose
  if(wOrientation == 62.0 || wOrientation == 65.0 || wOrientation == 69.0){
    tf2::Quaternion quaternion;
    quaternion.setRPY(target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z);
    quaternion.normalize();
    target_pose.orientation.x = quaternion.getX();
    target_pose.orientation.y = quaternion.getY();
    target_pose.orientation.z = quaternion.getZ();
    target_pose.orientation.w = quaternion.getW();
  }

  bool status;
  std::cout << "Pose published, wOrientation " <<  wOrientation << std::endl;

  if (wOrientation != 65.0 && wOrientation != 69.0){
    move_group.setPoseTarget(target_pose);
    // Sets position + orientation tolerance
    ROS_INFO_NAMED("move_to_pose", "Setting the target position to x=%g, y=%g, z=%g",target_pose.position.x, target_pose.position.y, target_pose.position.z);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    move_group.plan(my_plan);
    status = (move_group.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  }
  // Compute cartesian path if w is 65
  else {
    waypoints.push_back(target_pose);
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    double eef_step = 0.005;

    // Plan a maximum of 3 times before giving up
    double fraction;
    for (int i = 0; i < 5; i++){
      fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
      eef_step += 0.001;
      if (i != 0) std::cout << "Recalculating cartesian path! (" << i << " of 4). Acheived: %" << fraction * 100.0 << std::endl;
      if(fraction > 0.5) break; 
    } 
    ROS_INFO_NAMED("move_to_path", "Cartesian path (%.2f%% acheived)", fraction * 100.0);
    status = (move_group.execute(trajectory) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  }

  //Moves robot
  std_msgs::Bool move_status;
  move_status.data = status;
  pub.publish(move_status);
  std::cout << "Ready to listen" << std::endl;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "demo");
  ros::AsyncSpinner spinner(2);

  std::cout << "Set position tolerance: ";
  std::cin >> positionTolerance;
  
  std::cout << "Set orientation tolerance: ";
  std::cin >> orientationTolerance;

  spinner.start();

  //Initialize MoveIt!
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  const robot_state::JointModelGroup* joint_model_group =
    move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  spinner.stop();
  spinner.start();
  std::cout << "Ready to listen" << std::endl;
  ros::NodeHandle n;

  //Publisher for move command success return
  pub = n.advertise<std_msgs::Bool>("move_success", 1000);

  //Subscriber for movement poses
  ros::Subscriber sub = n.subscribe("edo_move", 1000, edomoveCallback);
  ros::waitForShutdown();

  return 0;
}
