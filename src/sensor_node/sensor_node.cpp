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
#include "sensor_publisher.h"
#include "topic_names.h"

#define NAME "sensor_node"

std::pair<std::string, image_publisher*> createSensorProcess(std::string subscribe){
	image_publisher* process = new sensor_publisher(subscribe);
	return std::pair<std::string, sensor_publisher*>(subscribe, process);
}

void threadUpdateTopics(){

	ROS_INFO_NAMED(NAME, "Starting Topic subscriber");
	ros::NodeHandle nh;
	std::map<std::string, sensor_publisher*> processing;
	std::vector<std::string> topics = getTopicsOfType("ladybug/sensors");

	while(nh.ok())
	{
		for (std::vector<std::string>::const_iterator it = topics.begin(); it != topics.end(); it++)
		{
			 if (processing.find(it->data()) == processing.end()) {
				//create new publisher
				processing.insert(createSensorProcess(it->data()));
			}
		}
		sleep(2);
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

  ros::MultiThreadedSpinner spinner(1);
  spinner.spin();


  ROS_INFO_NAMED(NAME, "STOPPING");
  return 0;
}
