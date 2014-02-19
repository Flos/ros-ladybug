/*
 * topic_name.cpp
 *
 *  Created on: 19.02.2014
 *      Author: fnolden
 */
#include "topic_names.h"


std::string
getReceiverSensorMsgTopicName(){
	return "/windows/ladybug5/record/ladybug_sensor";
}

std::string
getReceiverImageMsgTopicName(int cameraNr){
	std::stringstream topic;
	topic << "/windows/ladybug5/record/ladybug_image" << cameraNr;
	return topic.str();
}

std::string
getTopicName(int cameraNr){
	std::stringstream topic;
	topic << "/windows/ladybug5" << getCameraName(cameraNr);
	return topic.str();
}

std::string
getCameraName(int cameraNr){
	std::stringstream topic;
	topic << "/image" << cameraNr;
	return topic.str();
}


std::vector<std::string>
getNodeList()
{
  std::vector<std::string> all_nodes;
  ros::master::getNodes(all_nodes);
  for (std::vector<std::string>::const_iterator it = all_nodes.begin(); it != all_nodes.end(); it++)
  {
	  std::cout << it->data() << std::endl;
  }
  return all_nodes;
}

std::vector<std::string>
getTopicsOfType(std::string datatype)
{
  ros::master::V_TopicInfo topic_info;
  ros::master::getTopics(topic_info);

  std::vector<std::string> all_topics;
  for (ros::master::V_TopicInfo::const_iterator it = topic_info.begin(); it != topic_info.end(); it++)
  {
	  std::cout << it->name.c_str() << " datatype: " << it->datatype << " " << std::endl;
	  if(it->datatype.compare(datatype) == 0){
		  all_topics.push_back(it->name);
	  }
  }

  return all_topics;
}

