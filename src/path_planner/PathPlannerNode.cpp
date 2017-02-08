#include "PathPlannerNode.h"
#include <ros/ros.h>

using namespace std;

PathPlannerNode::PathPlannerNode()
{
	bbox_sub_topic = nh_.resolveName("curc_ardrone/bounding_box");
	command_topic = nh_.resolveName("curc_ardrone/com");

	std::string refresh_freq_str;
	ros::param::get("~refresh_freq", refresh_freq_str);
	if (refresh_freq_str.size() > 0)
		sscanf(refresh_freq_str.c_str(), "%d", &refresh_freq);
	else
		refresh_freq = 20;
	ROS_INFO("Set refresh_freq to " << refresh_freq << "Hz" << endl);

	bbox_sub = nh_.subscribe(bbox_sub_topic, 3, &PathPlannerNode::bboxCb, this);
	curc_ardrone_pub = nh_.advertise<std_msgs::String>(command_topic, 5);
}

PathPlannerNode::~PathPlannerNode()
{
}

void PathPlannerNode::bboxCb(const curc_ardrone::bounding_box bbox)
{
	// if bounding_box is valid
		

	// else
		// hover
}

void PathPlannerNode::publishCommand(string c) 
{
	std_msgs::String s;
	s.data = c.c_str();
	tum_ardrone_pub.publish(s);
}

void PathPlannerNode::Loop()
{
	ros::Rate pub_rate(refresh_freq);

	while (nh_.ok())
	{
		// ------- 1. get bounding box -------
		ros::spinOnce();
	}
}