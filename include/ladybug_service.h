#include <ros/ros.h>
#include <string>
#include "ladybug/send_command.h"
#include "zmq_service.h"

#define name "windows"

class Ladybug_service{
private:
	std::string connection;

	Zmq_service zmq_service;
	ros::ServiceServer ros_service;

	ros::NodeHandle* nh;
public:
	Ladybug_service();
	void init(ros::NodeHandle &nh);
	bool callback_srv(ladybug::send_command::Request &req, ladybug::send_command::Response &res);
};
