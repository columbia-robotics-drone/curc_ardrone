#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <ros/ros.h>
#include <time.h>
#include "curc_ardrone/bounding_box.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "bbox_generator");
	ros::NodeHandle nh_;

	ROS_INFO("Started curc_ardrone bbox_generator node.");
	curc_ardrone::bounding_box bbox;
	ros::Rate pub_rate(30);

	ros::Publisher bbox_pub = nh_.advertise<curc_ardrone::bounding_box>("curc_ardrone/bounding_box", 5);

	while(nh_.ok())
	{
		bbox.header.seq = (uint32_t) rand();
		bbox.header.stamp = time(0);
		bbox.header.frame_id = "sample_id";
		bbox.x = (uint32_t)rand() % 640;
		bbox.y = (uint32_t)rand() % 300;
		bbox.width = 40;
		bbox.height = 30;
		bbox.confidence = (float32) 0.6;

		bbox_pub.publish(bbox);
	}

	return 0;
}