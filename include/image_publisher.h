/*
 * image_publisher.h
 *
 *  Created on: 15.02.2014
 *      Author: fnolden
 */

#ifndef image_publisher_H_
#define image_publisher_H_
#include "ros/ros.h"
#include "ladybug/image.h"
#include "image_transport/image_transport.h"
#include "camera_info_manager/camera_info_manager.h"
#include "opencv_helper.h"
#include <boost/thread.hpp>
#include "topic_names.h"
#include <tf/transform_broadcaster.h>
#include "sensor_msgs/distortion_models.h"

class image_publisher {
public:
	image_publisher(std::string subscribe_topic);
	virtual ~image_publisher();
	void callback(const ladybug::image &input);
private:
  ros::NodeHandle n_;
  ros::Subscriber sub_;
  image_transport::ImageTransport *it_;
  image_transport::Publisher pub_;
  ros::Publisher pub_info_;
  tf::TransformBroadcaster br;
  tf::Transform transform;
  std::string subscribe_topic_;
  std::string publish_topic_;
  std::string camera_;
  sensor_msgs::CameraInfo cam_info_msg;
  camera_info_manager::CameraInfoManager *camera_service;
};

#endif /* image_publisher_H_ */
