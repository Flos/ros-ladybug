/*
 * colorSepToImage.cpp
 *
 *  Created on: 15.02.2014
 *      Author: fnolden
 */

#include "image_rectified_publisher.h"

image_rectified_publisher::image_rectified_publisher(std::string subscribe_topic) {
	//Topic to subscribe
	ROS_INFO("image_rectified_publisher created");
	subscribe_topic_ = subscribe_topic;
	publish_topic_ = subscribe_topic + "/rectified";

	unsigned long int pos = subscribe_topic.rfind('/');
	std::string baseTopic = subscribe_topic.substr(0, pos); //"

	pos = baseTopic.rfind('/');
	++pos; // set position after '/'
	std::string camera = baseTopic.substr(pos,baseTopic.size()-pos);
	std::string basePath = "/home/fnolden/src/catkin_ws/src/ladybug/src/rectification_map/calibration/13122828_";

	filename_map_x = basePath + camera +"_map_x.yaml";
	filename_map_y = basePath + camera +"_map_y.yaml";
	ROS_INFO("Subscribing\t%s", subscribe_topic.c_str());
	ROS_INFO("Publishing \t%s", publish_topic_.c_str());
	ROS_INFO("Loading rectification map x: %s",filename_map_x.c_str());
	loadMat(map_x, filename_map_x);
	ROS_INFO("map x: %s Dims: %d %d %d depth %d",filename_map_x.c_str(), map_x.cols, map_x.rows, map_x.dims, map_x.depth());

	ROS_INFO("Loading rectification map y: %s",filename_map_y.c_str());
	loadMat(map_y, filename_map_y);
	ROS_INFO("map y: %s Dims: %d %d %d depth %d",filename_map_y.c_str(), map_y.cols, map_y.rows, map_y.dims, map_y.depth());

	sub_ = n_.subscribe(subscribe_topic.c_str(), 1, &image_rectified_publisher::callback, this);
	it_ = NULL;
}

image_rectified_publisher::~image_rectified_publisher() {
	ROS_INFO_STREAM("closing node for topic" << subscribe_topic_);
	delete it_;
}

void
image_rectified_publisher::callback(const sensor_msgs::ImageConstPtr &message)
{
	if(it_ == NULL){
		// Create image transport
		it_ = new image_transport::ImageTransport(n_);
		pub_ = it_->advertise(publish_topic_, 1);
	}
	cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(message, sensor_msgs::image_encodings::BGR8);
	if(cv_ptr->image.cols < map_x.cols /* image is smaller than map, resize map */
			|| cv_ptr->image.rows < map_x.rows ){

		cv::Size img_size(map_x.cols, map_x.rows);

		ROS_WARN("Resizeing image: map is bigger than image");
		ROS_WARN("image: %d %d %d depth %d map: %d %d %d depth %d",cv_ptr->image.cols, cv_ptr->image.rows, cv_ptr->image.dims, cv_ptr->image.depth(), map_x.cols, map_x.rows, map_x.dims, map_x.depth());
		cv::resize(cv_ptr->image, cv_ptr->image, img_size);
		//cv::resize(map_y, map_y, img_size);
	}
//	else if( cv_ptr->image.cols >  map_x.cols /* image is bigger than map, reload map */
//			|| cv_ptr->image.rows > map_x.rows ){
//
//		ROS_WARN("Reloading maps: image is bigger than map");
//		ROS_WARN("image: %d %d %d depth %d map: %d %d %d depth %d",cv_ptr->image.cols, cv_ptr->image.rows, cv_ptr->image.dims, cv_ptr->image.depth(), map_x.cols, map_x.rows, map_x.dims, map_x.depth());
//		loadMat(map_x, filename_map_x);
//		loadMat(map_y, filename_map_y);
//	}
	pub_.publish(rectifyImage(cv_ptr, map_x, map_y));
}

