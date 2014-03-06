#include "ros/ros.h"
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
#include "image_rectified_publisher.h"
#include "topic_names.h"

#define NAME "rectification_map"

std::pair<std::string, image_rectified_publisher*> createImageRectificationProcess(std::string subscribe){
	image_rectified_publisher* process = new image_rectified_publisher(subscribe);
	return std::pair<std::string, image_rectified_publisher*>(subscribe, process);
}

void threadUpdateTopics(){

	ROS_INFO_NAMED(NAME, "Starting Topic subscriber");
	ros::NodeHandle nh;
	std::map<std::string, image_rectified_publisher*> processing;
	std::vector<std::string> topics;

	while(nh.ok())
	{
		topics = getTopicsOfType("sensor_msgs/Image", getTopicName(), "/image");
		for (std::vector<std::string>::const_iterator it = topics.begin(); it != topics.end(); it++)
		{
			 if (processing.find(it->data()) == processing.end()) {
				//create new publisher
				processing.insert(createImageRectificationProcess(it->data()));
			}
		}
		sleep(5);
	}

	while(!processing.empty()){
		  delete processing.begin()->second;
		  processing.erase(processing.begin());
	}
	ROS_ERROR_NAMED(NAME, "STOPPING Topic subscriber");
}

int main(int argc, char **argv){

  ros::init(argc, argv, NAME);
  ROS_INFO_STREAM("Starting " << NAME);

  new boost::thread(threadUpdateTopics);

  ros::MultiThreadedSpinner spinner(6);
  spinner.spin();


  ROS_INFO_NAMED(NAME, "STOPPING");
  return 0;
}
