/*
 * colorSepToImage.cpp
 *
 *  Created on: 15.02.2014
 *      Author: fnolden
 */

#include "image_publisher.h"

image_publisher::image_publisher(std::string subscribe_topic) {
	//Topic you want to subscribe
	subscribe_topic_ = subscribe_topic;

	ROS_INFO_STREAM("Creating processing node for topic" << subscribe_topic.c_str());
	sub_ = n_.subscribe(subscribe_topic.c_str(), 4, &image_publisher::callback, this);
	it_ = NULL;
}

image_publisher::~image_publisher() {
	ROS_INFO_STREAM("closing node for topic" << subscribe_topic_);
	delete it_;
}

void
image_publisher::callback(const ladybug::image &input)
{
	if(it_ == NULL){
		//Topic you want to publish
		it_ = new image_transport::ImageTransport(n_);
		pub_ = it_->advertise(getTopicName(input.camera_number), 4);
		transform.setOrigin( tf::Vector3(input.translation[0], input.translation[1], input.translation[2]) );
	    transform.setRotation( tf::Quaternion(input.rotation[0], input.rotation[1], input.rotation[2]) );
	    camera_ = getCameraName(input.camera_number);
	}
	br.sendTransform(tf::StampedTransform(transform, input.header.stamp, "ladybug_link", camera_ ));
	pub_.publish(createImgPtr(&input));
}

