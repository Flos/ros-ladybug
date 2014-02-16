#include "ros/ros.h"
#include "ladybug/image_colorsep.h"
#include <sstream>
#include <image_transport/image_transport.h>
#include "opencv_helper.h"
#include <string.h>
#include <map>
#include <boost/thread.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/bind.hpp>
#include <iostream>
#include <functional>
#include "helper.h"
#include "colorSepToImage.h"

#define NAME "image_node"
#define NUM_CAMERAS = 6

void getTopicName(int cameraNr, std::string &topic_in, std::string &topic_out){
	std::stringstream topic;
	topic << "/windows/ladybug5/LADYBUG_RAW_CAM" << cameraNr;
	topic_out = topic.str();
	topic << "/jpg_color_sep";
	topic_in = topic.str();
}

std::pair<std::string, colorSepToImage*> createImageProcess(int camera){
	std::string in;
	std::string out;
	getTopicName(camera, in, out);

	colorSepToImage* process = new colorSepToImage(in, out);
	return std::pair<std::string, colorSepToImage*>(in,process);
}

int main(int argc, char **argv){

  ros::init(argc, argv, NAME);
  ros::NodeHandle nh;

  std::map<std::string, colorSepToImage*> processing;

  for(unsigned int i = 0; i < 6; ++i){
	processing.insert(createImageProcess(i));
  }

  ros::MultiThreadedSpinner spinner(6);
  spinner.spin();


  while(!processing.empty()){
	  delete processing.begin()->second;
	  processing.erase(processing.begin());
  }

  ROS_INFO_STREAM_NAMED(NAME, "STOPPING");
  return 0;
}
