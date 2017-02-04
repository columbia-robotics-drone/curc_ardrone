#include <ros/ros.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "path_planner");
	
	ROS_INFO("Started curc_ardrone path_planner node.");

	return 0;
}