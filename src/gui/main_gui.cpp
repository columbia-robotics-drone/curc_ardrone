#include <ros/ros.h>
//TODO: see ros_opentld/src/ ui_baseFrame.h and related gui nodes. We should expand that gui with elements from TUM's (or rather, expand TUM's with elemetns from ros_opentld) and make the gui a node here
int main(int argc, char **argv)
{
	ros::init(argc, argv, "gui");
	
	ROS_INFO("Started curc_ardrone gui node.");

	return 0;
}
