#include "PathPlannerNode.h"
#include "std_msgs/String.h"
#include <math.h>
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
	cout << "Set refresh_freq to " << refresh_freq << "Hz" << endl;

	bbox_sub = nh_.subscribe(bbox_sub_topic, 3, &PathPlannerNode::bboxCb, this);
	curc_ardrone_pub = nh_.advertise<std_msgs::String>(command_topic, 5);
}

PathPlannerNode::~PathPlannerNode()
{
}

int PathPlannerNode::bboxIsValid(const curc_ardrone::bounding_box bbox)
{
	if (bbox.x != 1 && bbox.y != 1 && bbox.width != 1 && bbox.height != 1)
		return 1;
	else
		return 0;
}

void printErr(struct ErrStruct *err)
{
	cout << "f_x: " << err->f_x << endl;
	cout << "f_y: " << err->f_y << endl;
	cout << "f_delta: " << err->f_delta << endl;
}

struct ErrStruct *PathPlannerNode::calculateError(const curc_ardrone::bounding_box bbox)
{
	struct ErrStruct *err;
	// hardcoded image width/height--TODO: reorganize
	float w_im, h_im;

	w_im = 640;
	h_im = 300;

	err->f_x = (bbox.x + bbox.width/2)/w_im;
	err->f_y = (bbox.y + bbox.height/2)/h_im;
	err->f_delta = sqrt((w_im*h_im)/(bbox.width*bbox.height));

	printErr(err);	// debug

	return err;
}



void PathPlannerNode::bboxCb(const curc_ardrone::bounding_box bbox)
{
	cout << "bboxCb" << endl;
	int err;

	// if bounding_box is valid
	if (bboxIsValid(bbox)) 
	{
		err = calculateError(bbox);
	}

	// use error to control autopilot

}

void PathPlannerNode::publishCommand(string c) 
{
	std_msgs::String s;
	s.data = c.c_str();
	curc_ardrone_pub.publish(s);
}

void PathPlannerNode::Loop()
{
	ros::Rate pub_rate(refresh_freq);

	while (nh_.ok())
	{
		// ------- 1. get bounding box -------
		ros::spinOnce();

		// ------- 2. 
 	}
}