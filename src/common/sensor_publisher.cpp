/*
 * colorSepToImage.cpp
 *
 *  Created on: 15.02.2014
 *      Author: fnolden
 */

#include "sensor_publisher.h"

sensor_publisher::sensor_publisher(std::string subscribe_topic) {
	//Topic you want to subscribe
	hasPublisher = false;
	subscribe_topic_ = subscribe_topic;

	ROS_INFO_STREAM("Creating processing node for topic" << subscribe_topic.c_str());
	sub_ = n_.subscribe(subscribe_topic.c_str(), 4, &sensor_publisher::callback, this);
}

sensor_publisher::~sensor_publisher() {
	ROS_INFO_STREAM("closing node for topic" << subscribe_topic_);
}

void
sensor_publisher::callback(const ladybug::image &input)
{
	if(hasPublisher){
		//Topic you want to publish
		//publish_topic_ = getTopicName(input.camera_number);
		//pub_ = it_->advertise(publish_topic_, 4);
	}
	pub_.publish(createImgPtr(&input));
}

