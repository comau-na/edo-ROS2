 #!/bin/bash

. /opt/ros/melodic/setup.bash 
. ROS/devel/setup.bash 

echo "Neat! Topics will publish in the background for testing purposes!"

rostopic pub -r 10 /machine_state edo_core_msgs/MachineState '{current_state: 1, opcode: 4}' &
rostopic pub -r 10 /cartesian_pose edo_core_msgs/CartesianPose '{x: 1.00, y: 2.00, z: 1.00, a: 2.00, e: 1.00, r: 1.00}' &
rosrun edo_core edo_jnt_handler &

#rostopic pub /machine_state edo_core_msgs/MachineState '{current_state: 1, opcode: 4}'



