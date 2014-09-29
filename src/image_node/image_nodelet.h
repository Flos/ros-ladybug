/*
 * image_nodelet.h
 *
 *  Created on: 28.09.2014
 *      Author: fnolden
 */

#ifndef IMAGE_NODELET_H_
#define IMAGE_NODELET_H_

#include "nodelet/nodelet.h"
#include "ladybug/image.h"
#include "image_publisher.h"
#include "topic_names.h"
#include <string.h>
#include <map>
#include <boost/thread.hpp>
#include <boost/bind.hpp>

namespace ladybug {

class Image_nodelet : public nodelet::Nodelet {
public:
	Image_nodelet();
	virtual void onInit();
	virtual ~Image_nodelet();
private:
	void topic_listener();
	std::pair<std::string, image_publisher*> createImageProcess(std::string subscribe);
	std::string name;
	ros::NodeHandle nh;
	std::map<std::string, image_publisher*> processing;
	boost::shared_ptr<boost::thread> topic_listener_thread_;
};

} /* namespace ladybug */

#endif /* IMAGE_NODELET_H_ */
