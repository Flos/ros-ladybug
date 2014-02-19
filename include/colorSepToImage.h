/*
 * colorSepToImage.h
 *
 *  Created on: 15.02.2014
 *      Author: fnolden
 */

#ifndef colorSepToImage_H_
#define colorSepToImage_H_
#include "ros/ros.h"
#include "ladybug/image_colorsep.h"
#include "image_transport/image_transport.h"
#include "opencv_helper.h"
#include <boost/thread.hpp>

class colorSepToImage {
public:
	colorSepToImage(std::string subscribe_topic, std::string publish_topic);
	virtual ~colorSepToImage();
	void callback(const ladybug::image &input);
private:
  ros::NodeHandle n_;
  ros::Subscriber sub_;
  image_transport::ImageTransport *it_;
  image_transport::Publisher pub_;
  std::string topic_in_;
  std::string topic_out_;
};

#endif /* colorSepToImage_H_ */
