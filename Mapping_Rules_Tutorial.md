# How to create map custom messages manually using yaml
	
### When to make a custom mapping rule:

Most of the time, messages are automatically mapped by the dynamic bridge. If messages do not share the same name or have different field names, you must create a yaml file to tell the bridge to pair the messages and fields. It is sometimes necessary to have different names for the ROS and ROS2 versions of a message because ROS allows capital letters in message names while ROS2 does not.
	
### Format for mapping rules:
	
		-
		  ros1_package_name: 'edo_core_msgs'
		  ros1_message_name: 'JointState'
		  ros2_package_name: 'edo_core_msgs'
		  ros2_message_name: 'JointState'
		  fields_1_to_2:
		    position: 'position'
		    velocity: 'velocity'
		    current: 'current'
		    commandFlag: 'command_flag'
		    R_jnt: 'r_jnt'

		-
		  ros1_package_name: 'edo_core_msgs'
		  ros1_message_name: 'JointFwVersion'
		  ros2_package_name: 'edo_core_msgs'
		  ros2_message_name: 'JointFwVersion'
		  fields_1_to_2:
			id: 'id'
			majorRev: 'major_rev'
			minorRev: 'minor_rev'
			revision: 'revision'
			svn: 'svn'

**TIPS:**
	
- You have to start the file with a '-' 
- If you would like to map multiple messages in a file they must also be separated with a '-'
- If the names of the fields are the same, you do not have to include anything after "ros2_message_name"
- The mapping rule file must be of type .yaml
	
### How to use the mapping rule:
Your file structure should look like this
		
			.
			├─ ros1_msgs_ws
			│  └─ src
			│     └─ bridge_msgs
			│        └─ msg
			│           └─ JointCommand.msg
			├─ ros2_msgs_ws
			│  └─ src
			│     └─ bridge_msgs
			│        ├─ msg
			│        │  └─ JointCommand.msg
			│        └─ # YAML file of your custom interfaces have non-matching names
			└─ bridge_ws
			   └─ src
				  └─ ros1_bridge
					 └─ CMakeLists.txt # this is the cmakelist.txt that you have to edit
					 └─ package.xml # this is the package.xml file that you have to edit
		
Now rebuild your ROS2 message package using this command: colcon build --packages-select YOUR PACKAGE NAME

You must add these lines to the Package.xml file of the ros1_bridge package:

			<export>
				<build_type>ament_cmake</build_type>
				<ros1_bridge mapping_rules="mapping_rule.yaml"/>
			</export>

NOTE: there should already be a section in the package.xml that looks like this, add the extra line to the existing export tag: 

		<export>
			<build_type>ament_cmake</build_type>
		</export>

You must add these lines to the cmakelists.txt file of the ros1_bridge:

		install(
		  FILES mapping_rule.yaml
		  DESTINATION share/${PROJECT_NAME})

### Final step, rebuilding the bridge:
		
The general structure of how to build the bridge with custom messages is: 
1. Source ROS1
2. Source ROS2
3. Source ROS1 message package
4. Source ROS2 message package
5. Build the bridge
		
The specific commands for rebuilding will look similar to this:
			. /opt/ros/melodic/setup.bash
			. /opt/ros/eloquent/setup.bash
			. <workspace-parent-path>/ros1_msgs_ws/install_isolated/setup.bash
			. <workspace-parent-path>/ros2_msgs_ws/install/local_setup.bash
			cd <workspace-parent-path>/bridge_ws
			colcon build --packages-select ros1_bridge --cmake-force-configure
		
Now, run this command to ensure that the message was paired: ros2 run ros1_bridge dynamic_bridge --print-pairs
- your message should now show up on this list
		
