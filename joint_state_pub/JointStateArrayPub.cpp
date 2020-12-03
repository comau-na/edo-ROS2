#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include "edo_core_msgs/JointControl.h"
#include "edo_core_msgs/JointControlArray.h"
#include "edo_core_msgs/JointStateArray.h"
#include <iostream>
#include <cstdio>

static edo_core_msgs::JointStateArray states_;
double controller_frequency_;

void stateCB(const edo_core_msgs::JointControlArray::ConstPtr& msg)
{
	ROS_INFO("control received from algorithms j1 %f, j2 %f, j3 %f, j4 %f, j5 %f, j6 %f",msg->joints[0].position, msg->joints[1].position, msg->joints[2].position, msg->joints[3].position, msg->joints[4].position, msg->joints[5].position);

	//states_.size = 6;
	for(size_t i = 0; i < states_.joints.size(); i++)
	{
		states_.joints[i].position = msg->joints[i].position;
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "edo_jnt_handler");
	ROS_INFO("jnt handler node init...");
	ros::NodeHandle node_obj;
	ros::NodeHandle private_nh("~");

	ros::Publisher state_publisher = node_obj.advertise<edo_core_msgs::JointStateArray>("machine_algo_jnt_state", 100);
	ros::Subscriber control_subscriber = node_obj.subscribe("algo_jnt_ctrl", 100, stateCB);

	private_nh.param<double>("controller_frequency", controller_frequency_, 100);

	edo_core_msgs::JointState state[6];
	state[0].position = 1;
	state[0].velocity = 0;
	state[0].current = 0;

	state[1].position = 2;
	state[1].velocity = 0;
	state[1].current = 0;

	state[2].position = 90;//90;
	state[2].velocity = 0;
	state[2].current = 0;

	state[3].position = 0;
	state[3].velocity = 0;
	state[3].current = 0;

	state[4].position = 0;//90;
	state[4].velocity = 0;
	state[4].current = 0;

	state[5].position = 0;
	state[5].velocity = 0;
	state[5].current = 0;


	//states_.size = 6;
	states_.joints.resize(6);
	for (size_t i = 0; i < states_.joints.size(); i++) {
		states_.joints[i] = state[i];
	}
	state_publisher.publish(states_);
	//sleep(1.0);

	ros::Rate loop_rate(controller_frequency_);//100
	while (ros::ok())
	{
		ros::spinOnce();
    state_publisher.publish(states_);
		loop_rate.sleep();
	}
	return 0;
}