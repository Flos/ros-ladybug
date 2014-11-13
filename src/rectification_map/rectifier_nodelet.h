/*
 * image_publisher.h
 *
 *  Created on: 15.02.2014
 *      Author: fnolden
 */

#ifndef image_rectifier_nodelet_H_
#define image_rectifier_nodelet_H_
#include "ros/ros.h"
#include "ladybug/image.h"
#include "image_transport/image_transport.h"
#include "opencv_helper.h"
#include <boost/thread.hpp>
#include "topic_names.h"
#include <tf/transform_broadcaster.h>
#include "nodelet/nodelet.h"

namespace ladybug {

class Rectifier_nodelet : public nodelet::Nodelet {
public:
	Rectifier_nodelet();
	virtual void onInit();
	virtual ~Rectifier_nodelet();
	void callback(const sensor_msgs::ImageConstPtr &input);
private:
	ros::NodeHandle n_;
	ros::Subscriber sub_;
	image_transport::ImageTransport *it_;
	image_transport::Publisher pub_;
	std::string node_name_;
	std::string subscribe_topic_;
	std::string publish_topic_;
	std::string filepath_;
	cv::Mat map_x;
	cv::Mat map_y;
	void load_maps();
};

} /* namespace ladybug */
#endif /* image_rectified_publisher_H_ */
