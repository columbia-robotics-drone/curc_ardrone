#include <ros/ros.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "state_estimation");
	
	ROS_INFO("Started curc_ardrone state_estimation node.");

	return 0;
}