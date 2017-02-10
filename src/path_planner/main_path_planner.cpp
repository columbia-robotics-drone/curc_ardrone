#include <ros/ros.h>
#include "PathPlannerNode.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "curc_path_planner");
	
	ROS_INFO("Started curc_ardrone path_planner node.");

	PathPlannerNode path_planner;

	path_planner.Loop();

	return 0;
}