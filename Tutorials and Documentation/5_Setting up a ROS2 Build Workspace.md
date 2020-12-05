# Setting up a ROS2 Build Workspace

For this projecect there are two primary packages that you will use ROS1 for:
* edo_core_msgs - Same message package as ROS1, but with the syntax changed for ROS2
* edo_manual_ctrl- e.DO wrapper class used to control the e.DO robot arm using ROS2


1. To create a colcon (ROS2) workspace run the following commands
    
    `mkdir -p ~/<colcon_ws>/`
    `cd ~/<colcon_ws>/`
    `colcon build`

    This will create a 'build', 'install', and 'log' folder in your colcon workspace

2. Whenever you build a package in ROS2, you will need to run the following source command inside your colcon workspace:

    `. install/setup.bash`

Now that your workspace has been added to your path, you will be able to use your new package's executables