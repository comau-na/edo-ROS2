 #!/bin/bash

 . install/setup.bash 
 . /opt/ros/melodic/setup.bash 

echo "Neat! The bridge should run now!"
roscore & 
source ~/newgithub/edo-ROS2/install/setup.bash
sleep 1 


ros2 run ros1_bridge dynamic_bridge --bridge-all-topics 


