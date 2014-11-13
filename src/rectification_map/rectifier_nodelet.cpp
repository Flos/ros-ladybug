/*
 * rectifier_nodelet.cpp
 *
 *  Created on: 13.11.2014
 *      Author: fnolden
 */

#include "rectifier_nodelet.h"
#include <pluginlib/class_list_macros.h>

// watch the capitalization carefully
PLUGINLIB_DECLARE_CLASS(ladybug, Rectifier_nodelet, ladybug::Rectifier_nodelet, nodelet::Nodelet)

namespace ladybug {

Rectifier_nodelet::Rectifier_nodelet() {
	it_ = NULL;
}

void
Rectifier_nodelet::onInit(){
	// 1. Get name, topics and lookUpTable path
	n_.param<std::string>("name", node_name_ , "rectification_nodelet");
	n_.param<std::string>("lut", filepath_, "conf/calibration/13122828_");
	n_.param<std::string>("sub", subscribe_topic_, "/no_topic_selected");
	n_.param<std::string>("pub", publish_topic_, subscribe_topic_ + "_rect" );

	// 2. Info
	ROS_INFO_NAMED(node_name_, "image_rectifier nodelet created");
	ROS_INFO_NAMED(node_name_, "Subscribing\t%s", subscribe_topic_.c_str());
	ROS_INFO_NAMED(node_name_, "Publishing \t%s", publish_topic_.c_str());
	ROS_INFO_NAMED(node_name_, "Calibration file Path: \t%s", filepath_.c_str());

	// subscribe to topic
	sub_ = n_.subscribe(subscribe_topic_.c_str(), 1, &Rectifier_nodelet::callback, this);

	// load lut
	load_maps();
}

Rectifier_nodelet::~Rectifier_nodelet() {
	ROS_INFO_NAMED(node_name_, "closing node for topic %s", subscribe_topic_.c_str());
	delete it_;
}

void
Rectifier_nodelet::load_maps(){
	std::string filename_x = filepath_ + "x.yaml";
	std::string filename_y = filepath_ + "y.yaml";

	loadMat(map_x, filename_x);
	loadMat(map_y, filename_y);
}

void
Rectifier_nodelet::callback(const sensor_msgs::ImageConstPtr &message)
{
	cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(message, sensor_msgs::image_encodings::BGR8);
	if(it_ == NULL){
		// Create image transport
		it_ = new image_transport::ImageTransport(n_);
		pub_ = it_->advertise(publish_topic_, 1);

	}
	if(cv_ptr->image.cols != map_x.cols /* image and lut have different sizes */
			|| cv_ptr->image.rows != map_x.rows ){

		cv::Size img_size(map_x.cols, map_x.rows);

		ROS_WARN_NAMED(node_name_,"image and lut have different sizes: %d %d %d depth %d map: %d %d %d depth %d",cv_ptr->image.cols, cv_ptr->image.rows, cv_ptr->image.dims, cv_ptr->image.depth(), map_x.cols, map_x.rows, map_x.dims, map_x.depth());
		ROS_WARN_NAMED(node_name_,"reloading lut and resizing");

		load_maps();
		cv::resize(map_y, map_y, img_size, 0, 0, cv::INTER_CUBIC);
		cv::resize(map_x, map_x, img_size, 0, 0, cv::INTER_CUBIC);
	}
	if(pub_.getNumSubscribers()>0){
		pub_.publish(rectifyImage(cv_ptr, map_x, map_y));
	}
}

} /* namespace ladybug */
