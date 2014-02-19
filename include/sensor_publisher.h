/*
 * sensor_publisher.h
 *
 *  Created on: 15.02.2014
 *      Author: fnolden
 */

#ifndef sensor_publisher_H
#define sensor_publisher_H
#include "ros/ros.h"
#include "ladybug/sensors.h"
#include "image_transport/image_transport.h"
#include "opencv_helper.h"
#include <boost/thread.hpp>
#include "topic_names.h"

class sensor_publisher {
public:
	sensor_publisher(std::string subscribe_topic);
	virtual ~sensor_publisher();
	void callback(const ladybug::image &input);
private:
	ros::NodeHandle n_;
	ros::Subscriber sub_;
	ros::Publisher pub_;
	bool hasPublisher;
	std::string subscribe_topic_;
	std::string publish_topic_;
};

#endif /* sensor_publisher_H_ */
