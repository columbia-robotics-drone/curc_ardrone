#include <ros/ros.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "curc_tracker");
	
	ROS_INFO("Started curc_ardrone tracker node.");

	return 0;
}