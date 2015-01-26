#include "ros/ros.h"
#include "ladybug/image.h"
#include <sstream>
#include <image_transport/image_transport.h>
#include "opencv_helper.h"
#include <string.h>
#include <map>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <iostream>
#include <functional>
#include "helper.h"
#include "image_publisher.h"
#include "topic_names.h"

#define NAME "image_node"

int main(int argc, char **argv){

	ros::init(argc, argv, NAME);
	ros::NodeHandle node;
	ros::NodeHandle priv_nh("~");

	image_publisher image_publisher_node(node, priv_nh);

	ros::Rate rate(60);
	while(node.ok()){
		ros::spinOnce();
		rate.sleep();
	}

	ROS_INFO_NAMED(NAME, "STOPPING");
	return 0;
}
