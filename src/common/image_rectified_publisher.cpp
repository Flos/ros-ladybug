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
	publish_topic_ = getSubTopic(subscribe_topic) + "/image_rect_color";

	std::string baseTopic = getSubTopic(subscribe_topic);

	unsigned int pos = baseTopic.rfind('/');
	++pos; // set position after '/'
	camera_ = baseTopic.substr(pos,baseTopic.size()-pos);
	n_.param<std::string>("calibration_path", basePath_ , "/home/fnolden/src/catkin_ws/src/ladybug/src/rectification_map/calibration/13122828_");

	sub_ = n_.subscribe(subscribe_topic.c_str(), 1, &image_rectified_publisher::callback, this);
	it_ = NULL;

	ROS_INFO("Subscribing\t%s", subscribe_topic.c_str());
	ROS_INFO("Publishing \t%s", publish_topic_.c_str());
	ROS_INFO("Calibration file Path: \t%s", basePath_.c_str());
}

image_rectified_publisher::~image_rectified_publisher() {
	ROS_INFO_STREAM("closing node for topic" << subscribe_topic_);
	delete it_;
}

void
image_rectified_publisher::load_maps(int w, int h){
	char filename_x[255];
	char filename_y[255];
	sprintf(filename_x,"%s%s_h%d_w%d%s",basePath_.c_str(), camera_.c_str(), h, w, "_map_x.yaml");
	sprintf(filename_y,"%s%s_h%d_w%d%s",basePath_.c_str(), camera_.c_str(), h, w, "_map_y.yaml");

	filename_map_x_ = filename_x;
	filename_map_y_ = filename_y;

	ROS_INFO("Loading rectification map x: %s",filename_map_x_.c_str());
	loadMat(map_x, filename_map_x_);
	ROS_INFO("map x: %s Dims: %d %d %d depth %d",filename_map_x_.c_str(), map_x.cols, map_x.rows, map_x.dims, map_x.depth());

	ROS_INFO("Loading rectification map y: %s",filename_map_y_.c_str());
	loadMat(map_y, filename_map_y_);
	ROS_INFO("map y: %s Dims: %d %d %d depth %d",filename_map_y_.c_str(), map_y.cols, map_y.rows, map_y.dims, map_y.depth());

}

void
image_rectified_publisher::callback(const sensor_msgs::ImageConstPtr &message)
{
	cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(message, sensor_msgs::image_encodings::BGR8);
	if(it_ == NULL){
		// Create image transport
		it_ = new image_transport::ImageTransport(n_);
		pub_ = it_->advertise(publish_topic_, 1);
		load_maps(cv_ptr->image.cols,cv_ptr->image.rows);
	}
	if(cv_ptr->image.cols < map_x.cols /* image is smaller than map, resize map */
			|| cv_ptr->image.rows < map_x.rows ){

		cv::Size img_size(map_x.cols, map_x.rows);

		ROS_WARN("Resizeing image: map is bigger than image");
		ROS_WARN("image: %d %d %d depth %d map: %d %d %d depth %d",cv_ptr->image.cols, cv_ptr->image.rows, cv_ptr->image.dims, cv_ptr->image.depth(), map_x.cols, map_x.rows, map_x.dims, map_x.depth());
		cv::resize(cv_ptr->image, cv_ptr->image, img_size);
		//cv::resize(map_y, map_y, img_size, 0, 0, cv::INTER_CUBIC);
		//cv::resize(map_x, map_x, img_size, 0, 0, cv::INTER_CUBIC);
	}
//	else if( cv_ptr->image.cols >  map_x.cols /* image is bigger than map, reload map */
//			|| cv_ptr->image.rows > map_x.rows ){
//
//		ROS_WARN("Reloading maps: image is bigger than map");
//		ROS_WARN("image: %d %d %d depth %d map: %d %d %d depth %d",cv_ptr->image.cols, cv_ptr->image.rows, cv_ptr->image.dims, cv_ptr->image.depth(), map_x.cols, map_x.rows, map_x.dims, map_x.depth());
//		loadMat(map_x, filename_map_x);
//		loadMat(map_y, filename_map_y);
//	}
	if(pub_.getNumSubscribers()>0){
		pub_.publish(rectifyImage(cv_ptr, map_x, map_y));
	}
}

