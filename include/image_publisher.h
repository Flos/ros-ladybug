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
#include "debug.h"


struct Image_publisher_config{
  std::string subscribe_topic;
  std::string publish_topic;
  std::string publish_topic_info;
  std::string calibration;
  std::string frame_id;
  bool has_config;
};

class image_publisher {
public:
	image_publisher(ros::NodeHandle &nh, ros::NodeHandle &nh_private);
	virtual ~image_publisher();
	void callback(const ladybug::image &input);
private:
  ros::NodeHandle nh;
  ros::NodeHandle nh_private;

  ros::Subscriber sub;
  image_transport::Publisher pub;
  ros::Publisher pub_info;
  boost::shared_ptr<image_transport::ImageTransport> it;
  boost::shared_ptr<camera_info_manager::CameraInfoManager> camera_service;

  Image_publisher_config config;

  sensor_msgs::CameraInfo cam_info_msg;
};

#endif /* image_publisher_H_ */
