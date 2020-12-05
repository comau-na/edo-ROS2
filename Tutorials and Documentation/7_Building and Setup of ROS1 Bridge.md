# Building and Setup of ROS1 Bridge

## Create Directories

In our experience, it is easier to create three seperate workspaces(subfolders) to organize the setup and build of the e.DO Manual Control Wrapper Class.  This should include:
    1. ROS1 folder
    2. ROS2 folder
    3. ROS Bridge  

### ROS1 Folder
The ROS1 folder should contain the following packages within a 'src' subfolder
* eDO_core_msgs
* joint_state_pub

### ROS2 Folder
The ROS2 folder should contain the following packages(no 'src' folder for ROS2):
* edo_core_msgs
* edo_manual_ctrl

### ROS Bridge Folder
The bridge folder should contain the following packages within a 'src' subfolder
*ros1_bridge

Once you have your directory setup, you can proceed with the build.

*Note: it is possible to build directrly from the cloned edo-ROS2 GitHub directory, but it is best practice to create a separate folder for building, as to not corrupt any files in the edo-ROS2 directory*

## Build ROS1 Packages
1. Open up a new terminal and source your ROS1 distro
    `. /opt/ros/melodic/setup.bash`

2. Navigate to your ROS1 workspace described above
    Example: `~/ros1_bridge_sandbox/ros1_msgs_ws`

3. Check for any dependencies by running the following command
    `rosdep install --from-paths ~/ros1_bridge_sandbox/ros1_msgs_ws --ignore-src`

3. Build the ROS1 packages using the following command
    `catkin_make_isolated --install`
    
*Note: This will create three new folders in your workspace: build_isolated, devel_isolated, and install_isolated*

*Note: Need to remove any build, install, log files prior to running rosdep*

4. If both packages successfully build, you and move onto the next step
5. Source the newly create "devel_isolated" folder in the ROS1 directory
    `. devel_isolated/setup.bash`
6. Open up a a roscore in the background

## Build ROS2 Packages

1. Open up a new terminal
2. Source your ROS2 distro
 `. /opt/ros/eloquent/setup.bash`

3. Navigate to your ROS2 workspace

3. Build edo_core_msgs on ROS2
 `colcon build --packages-select edo_core_msgs`

*Note: This will create a build, install, and log folder in the directory*

4. After a successful build, source the newly create local_setup file in the insatll folder
` . install/local_setup.bash`


## Building the Bridge

1. Open up a third terminal
2. Navigate to your bridge workspace
3. Create a 'src' folder inside the bridge workspace
4. Copy the ros1_bridge folder from the edo-ROS GitHub root folder to the 'src' folder
5. Navigate back up to the bridge workspace
6. Source ROS1 and ROS2
    `. /opt/ros/melodic/setup.bash`
    `. /opt/ros/eloquent/setup.bash`
7. Source the messages packages that weere compiled above for both ROS1 and ROS2
Example from my directory, but this will be wherever you build your messages package:
  `. ~/ros1_bridge_sandbox/ros1_msgs_ws/install_isolated/setup.bash`
  `. ~/ros1_bridge_sandbox/ros2_msgs_ws/install/local_setup.bash`

8. Check for any dependencies by running the following command
    `rosdep install --from-paths ~/ros1_bridge_sandbox/ros1_msgs_ws --ignore-src`

    *Note: If you get an error saying rosdep keys cannot be resolved, use the following command:*
    `rosdep install --from-paths src --ignore-src --skip-keys python-wxtools --rosdistro eloquent -y`

9. Compile the ros1_bridge
    `colcon build --packages-select ros1_bridge --cmake-force-configure`

    *Note: You may get a warning after the build that "1 of hte mappings can not be generatred due to missing dependencies- edo_core_msgs/JointFwVersionArray <-> edo_core_msgs/JointFwVersionArray: - edo_core_msgs/JointFwVersion".  Currently, the wrapper class does not use this message, so there is no need to map it to ROS2*

10. Source the newly created local_setup file in the install folder
    `. install/local_setup.bash`

11. Test the mapping by printing out the paired messages
    ` ros2 run ros1_bridge dynamic_bridge --print-pairs`

    *Note: This should list out all the edo_core_msg mappings including the custom mapping rules.  Please check to make sure all mappings are included before proceeding*
    
