/*
 * colorSepToImage.cpp
 *
 *  Created on: 15.02.2014
 *      Author: fnolden
 */

#include "image_publisher.h"

image_publisher::image_publisher(ros::NodeHandle &nh, ros::NodeHandle &nh_private) {
	this->nh = nh;
	this->nh_private = nh_private;


	//subscribe_topic_ = subscribe_topic;
	nh_private.getParam("subscribe_topic", config.subscribe_topic);
	nh_private.getParam("frame_id", config.frame_id);
	nh_private.getParam("publish_topic", config.publish_topic);
	nh_private.getParam("publish_topic_info", config.publish_topic_info);
	nh_private.getParam("calibration", config.calibration);

	ROS_INFO("Image_publisher: %s",config.subscribe_topic.c_str());
	ROS_INFO("frame_id: %s",config.frame_id.c_str());
	ROS_INFO("publish_topic: %s",config.publish_topic.c_str());
	ROS_INFO("publish_topic_info: %s",config.publish_topic_info.c_str());
	ROS_INFO("calibration: %s",config.calibration.c_str());

	sub = nh.subscribe(config.subscribe_topic.c_str(), 1, &image_publisher::callback, this);
	it.reset(new image_transport::ImageTransport(nh));

	pub = it->advertise(config.publish_topic.c_str(), 1);
	pub_info = nh.advertise<sensor_msgs::CameraInfo>(config.publish_topic_info, 1);

	camera_service.reset(new camera_info_manager::CameraInfoManager(nh, config.frame_id));
	config.has_config = camera_service->loadCameraInfo(config.calibration);
}

image_publisher::~image_publisher() {
}

void
image_publisher::callback(const ladybug::image &input)
{
	if(!config.has_config)
	{
		// Create CameraInfo
		cam_info_msg.header = input.header;
		cam_info_msg.width = input.width;
		cam_info_msg.height = input.height;
		cam_info_msg.distortion_model = "NONE";
		//cam_info_msg.D[0] =
		cam_info_msg.K[0] = input.focalX;
		cam_info_msg.K[2] = input.centerX;
		cam_info_msg.K[4] = input.focalY;
		cam_info_msg.K[5] = input.centerY;
		cam_info_msg.K[8] = 1;

		cam_info_msg.P[0] = input.focalX;
		cam_info_msg.P[2] = input.centerX;
		cam_info_msg.P[5] = input.focalY;
		cam_info_msg.P[6] = input.centerY;
		cam_info_msg.P[10] = 1;
		camera_service->setCameraInfo(cam_info_msg);
	}


	sensor_msgs::CameraInfo caminfo = camera_service->getCameraInfo();

	caminfo.header = input.header;

	if(pub.getNumSubscribers()>0){
		sensor_msgs::ImagePtr img_msg = createImgPtr(&input);

		if(!config.frame_id.empty()){
			caminfo.header.frame_id = config.frame_id;
			img_msg->header.frame_id = config.frame_id;
		}

		pub_info.publish(caminfo);
		pub.publish(img_msg);
		debug_header("image_publisher", caminfo.header);
	}
}

