# Setting up a ROS1 Build Workspace

For this projecect there are two primary packages that you will use ROS1 for:
* eDO_core_msgs - These are the messages used by e.DO to publish key messages on the ROS network.  In this project these need to be built on both the ROS1 and the ROS2 side as they will be bridged between the edo Core Package (e.DO source code) and the ROS2 wrapper class
* joint_state_pub - This will only be required when running a terminal simulation without usage of the e.DO hardware.  This is needed to publish JointState and JointStateArray messages which are required for the wrapper class to function properly in a simulated environment


1. To create a catkin workspace run the following commands
    
    `mkdir -p ~/<catkin_ws>/src`
    `cd ~/<catkin_ws>/`
    `catkin_make`

    This will create a CMakeLists.txt file in your 'src' folder.  In addition, you shoul have a 'build' and 'devel' folder.  In the 'devel' folder there will be several setup.*sh files.  You will need to sources these files to overlay this workspace on top of your environment.

2. Whenever you build a package in ROS1, you will need to run the following source command:

    `source devel/setup.bash`