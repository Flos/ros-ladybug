#include "ros/ros.h"
#include "ladybug/image.h"
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
	topic << "/ladybug_image";
	topic_in = topic.str();
}

std::pair<std::string, colorSepToImage*> createImageProcess(int camera){
	std::string in;
	std::string out;
	getTopicName(camera, in, out);

	colorSepToImage* process = new colorSepToImage(in, out+"/image");
	return std::pair<std::string, colorSepToImage*>(in,process);
}

int main(int argc, char **argv){

  ros::init(argc, argv, NAME);
  ros::NodeHandle nh;

  std::map<std::string, colorSepToImage*> processing;

  for(unsigned int i = 0; i < 6; ++i){
	processing.insert(createImageProcess(i));
  }

  colorSepToImage* process = new colorSepToImage("/windows/ladybug5/LADYBUG_PANORAMIC/ladybug_image","/windows/ladybug5/LADYBUG_PANORAMIC/image");
  processing.insert(std::pair<std::string, colorSepToImage*>("/windows/ladybug5/LADYBUG_PANORAMIC/ladybug_image",process));

  ros::MultiThreadedSpinner spinner(6);
  spinner.spin();


  while(!processing.empty()){
	  delete processing.begin()->second;
	  processing.erase(processing.begin());
  }

  ROS_INFO_STREAM_NAMED(NAME, "STOPPING");
  return 0;
}
