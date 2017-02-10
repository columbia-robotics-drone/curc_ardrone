#pragma once

#include <ros/ros.h>
#include "curc_ardrone/bounding_box.h"

#ifndef __PATHPLANNERNODE_H
#define __PATHPLANNERNODE_H

struct ErrStruct {
	float f_x;
	float f_y;
	float f_delta;
}

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
	std::string command_topic;

public:
	// filter bboxes for smoothness - TODO

	PathPlannerNode();
	~PathPlannerNode();

	int bboxIsValid(const curc_ardrone::bounding_box bbox);

	struct err_struct calculateError(curc_ardrone::bounding_box bbox);

	// ROS message callbacks
	void bboxCb(const curc_ardrone::bounding_box bbox);

	// main path-planning loop
	void Loop();

	// writes a string message to "/curc_ardrone/com"
	void publishCommand(std::string c);

};

#endif