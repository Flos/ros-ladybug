/*
 * image_publisher.h
 *
 *  Created on: 15.02.2014
 *      Author: fnolden
 */

#ifndef image_rectified_publisher_H_
#define image_rectified_publisher_H_
#include "ros/ros.h"
#include "ladybug/image.h"
#include "image_transport/image_transport.h"
#include "opencv_helper.h"
#include <boost/thread.hpp>
#include "topic_names.h"
#include <tf/transform_broadcaster.h>

class image_rectified_publisher {
public:
	image_rectified_publisher(std::string subscribe_topic);
	virtual ~image_rectified_publisher();
	void callback(const sensor_msgs::ImageConstPtr &input);
private:
  ros::NodeHandle n_;
  ros::Subscriber sub_;
  image_transport::ImageTransport *it_;
  image_transport::Publisher pub_;
  std::string subscribe_topic_;
  std::string publish_topic_;
  std::string camera_;
  std::string filename_map_x_;
  std::string filename_map_y_;
  std::string basePath_;
  cv::Mat map_x;
  cv::Mat map_y;
  void load_maps(int h, int w);
};

#endif /* image_rectified_publisher_H_ */
