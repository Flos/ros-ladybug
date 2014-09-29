/*
 * image_nodelet.cpp
 *
 *  Created on: 28.09.2014
 *      Author: fnolden
 */

#include "image_nodelet.h"
#include <pluginlib/class_list_macros.h>

// watch the capitalization carefully
PLUGINLIB_DECLARE_CLASS(ladybug, Image_nodelet, ladybug::Image_nodelet, nodelet::Nodelet)

namespace ladybug {

Image_nodelet::Image_nodelet() {
	// TODO Auto-generated constructor stub

}

void
Image_nodelet::onInit(){
	nh = getMTPrivateNodeHandle();
	topic_listener_thread_ = boost::shared_ptr<boost::thread>(new boost::thread( boost::bind( &Image_nodelet::topic_listener, this )));
	name = "image_nodelet";
//    ros::MultiThreadedSpinner spinner(0);
//    spinner.spin();
}

std::pair<std::string, image_publisher*>
Image_nodelet::createImageProcess(std::string subscribe){
	image_publisher* process = new image_publisher(subscribe);
	return std::pair<std::string, image_publisher*>(subscribe, process);
}

void
Image_nodelet::topic_listener(){
	ROS_INFO_NAMED(name, "Starting Topic subscriber");

		while(nh.ok())
		{
			std::vector<std::string> topics = getTopicsOfType("ladybug/image");
			for (std::vector<std::string>::const_iterator it = topics.begin(); it != topics.end(); it++)
			{
				 if (processing.find(it->data()) == processing.end()) {
					//create new publisher
					processing.insert(createImageProcess(it->data()));
				}
			}
			sleep(2);
		}

		while(!processing.empty()){
			  delete processing.begin()->second;
			  processing.erase(processing.begin());
		}
		ROS_ERROR_NAMED(name, "STOPPING Topic subscriber");
}

Image_nodelet::~Image_nodelet() {
	// TODO Auto-generated destructor stub
}

} /* namespace ladybug */
