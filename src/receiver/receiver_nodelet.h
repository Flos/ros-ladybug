/*
 * Receivernodelet.h
 *
 *  Created on: 27.09.2014
 *      Author: fnolden
 */
#include "nodelet/nodelet.h"
#include "ladybug/image.h"
#include "ladybug/sensors.h"
#include "zmq_service.h"
#include "opencv_helper.h"
#include <string.h>
#include "helper.h"
#include <sensor_msgs/image_encodings.h>
#include "topic_names.h"
#include <boost/thread.hpp>
#include "debug.h"

#ifndef RECEIVERNODELET_H_
#define RECEIVERNODELET_H_

namespace ladybug
{
	class Receiver_nodelet : public nodelet::Nodelet{
	public:
		void loop();
		virtual void onInit();
		virtual ~Receiver_nodelet();
	private:
		void publish_image(int type,ladybug::imagePtr &msg);
		void handle_message(ladybug5_network::pbMessage &recv_msg);
		std::string name;
		int max_time_diff;
		std::string connection;
		unsigned int sequence;
		Zmq_service zmq_service;
		ros::NodeHandle nh;
		std::map<std::string, ros::Publisher*> publisher_map;
		ros::Publisher* sensor_raw_publisher;
		boost::shared_ptr<boost::thread> thread_;
	};
}
#endif /* RECEIVERNODELET_H_ */
