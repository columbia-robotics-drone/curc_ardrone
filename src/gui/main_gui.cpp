#include <ros/ros.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "curc_gui");
	
	ROS_INFO("Started curc_ardrone gui node.");

	return 0;
}