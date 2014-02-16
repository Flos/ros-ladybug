/*
 * colorSepToImage.cpp
 *
 *  Created on: 15.02.2014
 *      Author: fnolden
 */

#include "colorSepToImage.h"

colorSepToImage::colorSepToImage(std::string subscribe_topic, std::string publish_topic) {
	//Topic you want to subscribe
	topic_in_ = subscribe_topic;
	topic_out_ = publish_topic;

	ROS_INFO_STREAM("Creating processing node for topic" << subscribe_topic.c_str());
	sub_ = n_.subscribe(subscribe_topic.c_str(), 4, &colorSepToImage::callback, this);

	//Topic you want to publish
	it_ = new image_transport::ImageTransport(n_);
	pub_ = it_->advertise(publish_topic.c_str() , 4);
}

colorSepToImage::~colorSepToImage() {
	ROS_INFO_STREAM("closing node for topic" << topic_in_);
	delete it_;
}

void
colorSepToImage::callback(const ladybug::image_colorsep &input)
{
	pub_.publish(createImgPtr(&input));
}

