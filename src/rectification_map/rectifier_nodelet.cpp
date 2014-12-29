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
	zoom_factor_ = 1;
}

void
Rectifier_nodelet::onInit(){
	// 1. Get name, topics and lookUpTable path
	n_ = getPrivateNodeHandle();
	n_.param<std::string>("name", node_name_ , "rectification_nodelet");
	n_.param<std::string>("lut", filepath_, "conf/calibration/13122828_");
	n_.param<std::string>("sub", subscribe_topic_, "/no_topic_selected");
	n_.param<std::string>("pub", publish_topic_, subscribe_topic_ + "_rect" );

	n_.param<double>("zoom", zoom_factor_, 1);
	n_.param<std::string>("sub_info", subscriber_info_topic_, subscribe_topic_ + "_info");
	n_.param<std::string>("pub_info", publish_info_topic_, publish_topic_ + "_info" );


	// 2. Info
	ROS_INFO_NAMED(node_name_, "name: \t%s", node_name_.c_str());
	ROS_INFO_NAMED(node_name_, "lut: \t%s", filepath_.c_str());
	ROS_INFO_NAMED(node_name_, "sub: \t%s", subscribe_topic_.c_str());
	ROS_INFO_NAMED(node_name_, "pub: \t%s", publish_topic_.c_str());

	ROS_INFO_NAMED(node_name_, "zoom: \t%f", zoom_factor_);
	ROS_INFO_NAMED(node_name_, "sub_info: \t%s", subscriber_info_topic_.c_str());
	ROS_INFO_NAMED(node_name_, "pub_info: \t%s", publish_info_topic_.c_str());


	// subscribe to topic
	sub_ = n_.subscribe(subscribe_topic_.c_str(), 1, &Rectifier_nodelet::callback, this);
	sub_info_ = n_.subscribe(subscriber_info_topic_.c_str(), 1, &Rectifier_nodelet::callback_camera_info, this);

	pub_info_ = n_.advertise<sensor_msgs::CameraInfo>(publish_info_topic_, 1);

	// load lut
	load_maps(zoom_factor_);
}

Rectifier_nodelet::~Rectifier_nodelet() {
	ROS_INFO_NAMED(node_name_, "closing node for topic %s", subscribe_topic_.c_str());
	delete it_;
}

void
Rectifier_nodelet::load_maps(double zoom_faktor){
	std::string filename_x = filepath_ + "x.yaml";
	std::string filename_y = filepath_ + "y.yaml";

	loadMat(map_x, filename_x);
	loadMat(map_y, filename_y);

	if(zoom_faktor > 1.0){
		// Resize maps to zoom to the center
		int width = map_x.cols;
		int height = map_y.rows;

		cv::Size img_size(map_x.cols*zoom_faktor, map_x.rows*zoom_faktor);
		cv::resize(map_y, map_y, img_size, 0, 0, cv::INTER_LINEAR);
		cv::resize(map_x, map_x, img_size, 0, 0, cv::INTER_LINEAR);

		//calculate borders to cut the image propperly
		int border_left =  (map_x.cols - width)/2;
		int border_top =   (map_y.rows - height)/2;

		//ROS_WARN("X: %i, Y: %i, width: %i, height: %i, Img width: %i height: %i", border_left, border_top, width, height, map_x.cols, map_x.rows);

		cv::Rect border = cv::Rect(border_left, border_top, width, height);
		map_x = map_x(border);
		map_y = map_y(border);
	}
	else if(zoom_faktor < 1.0){
		ROS_WARN_NAMED(node_name_,"zoom faktor is set to %f, but must be >= 1.0, reset zoom_faktor to 1.0", zoom_faktor);
		zoom_factor_ = 1.0;
	}

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

		load_maps(zoom_factor_);
		cv::resize(map_y, map_y, img_size, 0, 0, cv::INTER_CUBIC);
		cv::resize(map_x, map_x, img_size, 0, 0, cv::INTER_CUBIC);
	}
	if(pub_.getNumSubscribers()>0){
		pub_.publish(rectifyImage(cv_ptr, map_x, map_y));
	}
}

void
Rectifier_nodelet::callback_camera_info(const sensor_msgs::CameraInfo &input_cam_info){
	//Update projection matrix
	cam_info_msg = input_cam_info;
	//cam_info_msg =
	cam_info_msg.header = input_cam_info.header;

	//focal length
	cam_info_msg.P[0] = input_cam_info.P[0] * zoom_factor_; //fx
	cam_info_msg.P[5] = input_cam_info.P[5] * zoom_factor_; //fy

	//calculate principle point in new image
	int image_width_zoomed = input_cam_info.width  * zoom_factor_;
	int image_height_zoomed = input_cam_info.height  * zoom_factor_;
	int border_left =  MAX(0,(image_width_zoomed - input_cam_info.width)/2);
	int border_top =   MAX(0,(image_height_zoomed - input_cam_info.height)/2);

	ROS_INFO_NAMED(node_name_, "border_left: \t%i\t border_top: \t%i", border_left, border_top);

	cam_info_msg.P[2] = (input_cam_info.P[2] * zoom_factor_) - border_left; //cx
	cam_info_msg.P[6] = (input_cam_info.P[6] * zoom_factor_) - border_top; //cy

	pub_info_.publish(cam_info_msg);
}

} /* namespace ladybug */
