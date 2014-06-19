/*
 * colorSepToImage.cpp
 *
 *  Created on: 15.02.2014
 *      Author: fnolden
 */

#include "image_publisher.h"

image_publisher::image_publisher(std::string subscribe_topic) {
	subscribe_topic_ = subscribe_topic;

	publish_topic_ = getSubTopic(subscribe_topic_)+getRawImageName();

	ROS_INFO("Creating node for topic %s -> %s ",subscribe_topic.c_str(),publish_topic_.c_str());
	n_ = ros::NodeHandle(getSubTopic(subscribe_topic_));

	sub_ = n_.subscribe(subscribe_topic.c_str(), 1, &image_publisher::callback, this);
	it_ = NULL;
	camera_service = NULL;
}

image_publisher::~image_publisher() {
	ROS_INFO_STREAM("closing node for topic" << subscribe_topic_);
	delete it_;
	delete camera_service;
}

void
image_publisher::callback(const ladybug::image &input)
{
	if(it_ == NULL){
		// Create camera_service
		char name[255];
		std::sprintf(name,"ladybug_%s_camera%s_h%d_w%d", input.serial_number.c_str(), getCameraName(input.image_type).c_str(),  input.height, input.width);
		ROS_INFO_STREAM("Creating camera service: " << name);
		camera_service = new camera_info_manager::CameraInfoManager(n_, name);
		pub_info_ = n_.advertise<sensor_msgs::CameraInfo>(getSubTopic(subscribe_topic_)+"/camera_info", 1);

		// Create image transport
		it_ = new image_transport::ImageTransport(n_);
		pub_ = it_->advertise(publish_topic_, 1);

		// Create Transform
		tf::Quaternion quat;
		quat.setRPY(input.rotationX, input.rotationY, input.rotationZ);
		transform.setOrigin( tf::Vector3(input.translationX, input.translationY, input.translationZ) );
		transform.setRotation(quat);

//		// Create CameraInfo
//		cam_info_msg.header = input.header;
//		cam_info_msg.width = input.width;
//		cam_info_msg.height = input.height;
//		cam_info_msg.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
//		//cam_info_msg.D[0] =
//		cam_info_msg.K[0] = input.focalX;
//		cam_info_msg.K[2] = input.centerX;
//		cam_info_msg.K[4] = input.focalY;
//		cam_info_msg.K[5] = input.centerY;
//		cam_info_msg.K[8] = 1;
	}

	br.sendTransform(tf::StampedTransform(transform, input.header.stamp, "ladybug_link", input.header.frame_id));
	sensor_msgs::CameraInfo caminfo = camera_service->getCameraInfo();
	caminfo.header = input.header;
	pub_info_.publish(caminfo);

	if(pub_.getNumSubscribers()>0){
		pub_.publish(createImgPtr(&input));
	}
}

