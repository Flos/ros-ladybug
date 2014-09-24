#include <ros/ros.h>
#include <string>
#include "ladybug/send_command.h"
#include "zmq_service.h"
#include "ladybug_service.h"

int
main(int argc, char** argv)
{

  // Initialize ROS
  ros::init (argc, argv, name);
  ros::NodeHandle n("~");

  Ladybug_service service;
  service.init(n);

  ros::spin();
}
