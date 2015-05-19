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
	rotate_ = 0;
	calculated_rectification_parameters = false;
	cx = 0;
	cy =0;
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
	n_.param<std::string>("frame_id", frame_id_, "");
	n_.param<bool>("rotate", rotate_, 1);

	// 2. Info
	ROS_INFO_NAMED(node_name_, "name: \t%s", node_name_.c_str());
	ROS_INFO_NAMED(node_name_, "lut: \t%s", filepath_.c_str());
	ROS_INFO_NAMED(node_name_, "sub: \t%s", subscribe_topic_.c_str());
	ROS_INFO_NAMED(node_name_, "pub: \t%s", publish_topic_.c_str());

	ROS_INFO_NAMED(node_name_, "zoom: \t%f", zoom_factor_);
	ROS_INFO_NAMED(node_name_, "sub_info: \t%s", subscriber_info_topic_.c_str());
	ROS_INFO_NAMED(node_name_, "pub_info: \t%s", publish_info_topic_.c_str());


	// load lut
	load_maps();

	// subscribe to topic
	sub_ = n_.subscribe(subscribe_topic_.c_str(), 1, &Rectifier_nodelet::callback, this);
	sub_info_ = n_.subscribe(subscriber_info_topic_.c_str(), 1, &Rectifier_nodelet::callback_camera_info, this);

	pub_info_ = n_.advertise<sensor_msgs::CameraInfo>(publish_info_topic_, 1);

	 if(zoom_factor_ < 1.0){
		ROS_WARN_NAMED(node_name_,"zoom faktor is set to %f, but must be >= 1.0, reset zoom_faktor to 1.0", zoom_factor_);
		zoom_factor_ = 1.0;
	}


}

Rectifier_nodelet::~Rectifier_nodelet() {
	ROS_INFO_NAMED(node_name_, "closing node for topic %s", subscribe_topic_.c_str());
	delete it_;
}

void Rectifier_nodelet::zoom_image(cv::Point2d begin_rect,
		cv::Size& img_size, const cv::Mat &in, cv::Mat &out) {

	cv::Mat tmp;
	//printf("x: %f y: %f width: %d height: %d in.cols, %d in.rows %d\n", begin_rect.x, begin_rect.y, img_size.width, img_size.height, in.cols, in.rows);
	cv::resize(in, tmp, img_size, 0, 0, cv::INTER_LINEAR);
	cv::Rect border = cv::Rect(begin_rect.x, begin_rect.y, in.cols, in.rows);
	out = tmp(border);
}

void
Rectifier_nodelet::load_maps(){
	std::string filename_x = filepath_ + "x.yaml";
	std::string filename_y = filepath_ + "y.yaml";

	loadMat(map_x, filename_x);
	loadMat(map_y, filename_y);
}

void Rectifier_nodelet::find_point(int max, cv::Mat& rectified_point,
		cv::Point2d& new_c) {
	for (int r = 1; r+1 < rectified_point.rows; ++r) {
		for (int c = 1; c+1 < rectified_point.cols; ++c) {
			int bucket = 0;
			bucket+= rectified_point.at<cv::Vec3b>(cv::Point2d(c-1, r-1))[0];
			bucket+= rectified_point.at<cv::Vec3b>(cv::Point2d(c, r-1))[0];
			bucket+= rectified_point.at<cv::Vec3b>(cv::Point2d(c+1, r-1))[0];

			bucket+= rectified_point.at<cv::Vec3b>(cv::Point2d(c-1, r))[0];
			bucket+= rectified_point.at<cv::Vec3b>(cv::Point2d(c, r))[0];
			bucket+= rectified_point.at<cv::Vec3b>(cv::Point2d(c+1, r))[0];


			bucket+= rectified_point.at<cv::Vec3b>(cv::Point2d(c-1, r+1))[0];
			bucket+= rectified_point.at<cv::Vec3b>(cv::Point2d(c, r+1))[0];
			bucket+= rectified_point.at<cv::Vec3b>(cv::Point2d(c+1, r+1))[0];

			if (bucket > max) {
				new_c = cv::Point2d(c, r);
				max = bucket;
			}
		}
	}
}

void
Rectifier_nodelet::calc_parameters(const sensor_msgs::CameraInfo &caminfo){
	// 1: cx, cy mark
	init_lock.try_lock();

	cv::Mat rectified_point;
	cv::Point2d center = cv::Point2d(caminfo.P[2], caminfo.P[6]);
	cv::Mat empty = cv::Mat::zeros(map_x.rows, map_x.cols, CV_8UC3);

	cv::circle(empty,center,1,cv::Scalar(100,0,0),-1);
	//cv::circle(empty,center,1,cv::Scalar(255,0,0),-1);
	empty.at<cv::Vec3b>(center)[0] = 255;
//
//	// 2: zoom image
	if(zoom_factor_ > 1.0){
		cv::Point2d begin_rect(center.x*zoom_factor_-center.x, center.y*zoom_factor_-center.y);
		cv::Size img_size(map_x.cols * zoom_factor_, map_x.rows * zoom_factor_);
//
//		std::cout << "3";
		zoom_image(begin_rect, img_size, empty, rectified_point);
		zoom_image(begin_rect, img_size, map_x, map_x);
		zoom_image(begin_rect, img_size, map_y, map_y);
//		std::cout << "4";
	}else
	{
		rectified_point = empty;
	}

//	// 3: find point in image
	int max = -9999999;
	cv::Point2d new_c;

	if(rotate_){
		rotate90(rectified_point, rectified_point);
	}

	find_point(max, rectified_point, new_c);

	cx = new_c.x;
	cy = new_c.y;
//
//	if(new_c.x == 0 || new_c.y == 0){
//		cv::imwrite("/tmp/no_point.jpg", rectified_point);
//		printf("No point found... calc_param: cx: %f -> %f cy: %f -> %f topic: %s\n", center.x , new_c.x, center.y, new_c.y, subscriber_info_topic_.c_str());
//		return;
//	}
//
//
	cv::circle(rectified_point, new_c, 1, cv::Scalar(0,0,255), -1);
	cv::circle(rectified_point, center, 1, cv::Scalar(0,255,0), -1);

//	cv::imwrite("/tmp/calc_parameters.jpg", rectified_point);
	printf("calc_param: cx: %f -> %f cy: %f -> %f\n", center.x , new_c.x, center.y, new_c.y);
	calculated_rectification_parameters = true;
	init_lock.unlock();
}


void
Rectifier_nodelet::callback(const sensor_msgs::ImageConstPtr &message)
{
	if(!calculated_rectification_parameters){
		return;
	}
	cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(message, message->encoding);
	if(it_ == NULL){
		// Create image transport
		it_ = new image_transport::ImageTransport(n_);
		pub_ = it_->advertise(publish_topic_, 1);
	}
	if(cv_ptr->image.cols != map_x.cols /* image and lut have different sizes */
			|| cv_ptr->image.rows != map_x.rows ){

		cv::Size img_size(map_x.cols, map_x.rows);

		ROS_ERROR_NAMED(node_name_,"image and lut have different sizes: %d %d %d depth %d map: %d %d %d depth %d",cv_ptr->image.cols, cv_ptr->image.rows, cv_ptr->image.dims, cv_ptr->image.depth(), map_x.cols, map_x.rows, map_x.dims, map_x.depth());
		return;
	}
	if(pub_.getNumSubscribers()>0){
		cv_ptr->header.frame_id = frame_id_;
		pub_.publish(rectifyImage(cv_ptr, map_x, map_y, rotate_));
	}
}

void
Rectifier_nodelet::callback_camera_info(const sensor_msgs::CameraInfo &input_cam_info){


	if(input_cam_info.P[2] == 0 || input_cam_info.P[6] == 0 || input_cam_info.P[0] == 0 || input_cam_info.P[5] == 0){
		ROS_WARN_NAMED(node_name_,"Received empty cam info msg on topic: %s", subscriber_info_topic_.c_str());
		return;
	}

	cam_info_msg = input_cam_info;

	if(!calculated_rectification_parameters){
		calc_parameters(cam_info_msg);
	}

	//Update projection matrix
	cam_info_msg.header.frame_id = frame_id_;

	//focal length
	cam_info_msg.P[0] = (double)cam_info_msg.P[0] * zoom_factor_; //fx
	cam_info_msg.P[5] = (double)cam_info_msg.P[5] * zoom_factor_; //fy

	cam_info_msg.P[2] = cx;
	cam_info_msg.P[6] = cy;

	if(rotate_){
		cam_info_msg.width = map_x.rows;
		cam_info_msg.height = map_x.cols;
	}
	else{
		cam_info_msg.width = map_x.cols;
		cam_info_msg.height = map_x.rows;
	}

	pub_info_.publish(cam_info_msg);
}

} /* namespace ladybug */
