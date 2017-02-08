#pragma once

#include <ros/ros.h>
#include "curc_ardrone/bounding_box.h"

#ifndef __PATHPLANNERNODE_H
#define __PATHPLANNERNODE_H

class PathPlannerNode
{
private:
	// comm with tracker
	ros::Subscriber bbox_sub;

	// comm with autopilot
	ros::Publisher curc_ardrone_pub;

	ros::NodeHandle nh_;

	// parameters
	int refresh_freq;

	// resolve names
	std::string bbox_sub_topic;
	std::string curc_ardrone_sub_topic;
	std::string curc_ardrone_pub_topic;

public:
	// filter - TODO

	PathPlannerNode();
	~PathPlannerNode();

	// ROS message callbacks
	void bboxCb(const curc_ardrone::bounding_box bbox);

	// main path-planning loop
	void Loop();

	// writes a string message to "/curc_ardrone/com"
	void publishCommand(std::string c);

}

#endif