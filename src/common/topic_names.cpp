/*
 * topic_name.cpp
 *
 *  Created on: 19.02.2014
 *      Author: fnolden
 */
#include "topic_names.h"

std::string
getTopicName(){
	return "/ladybug5";
}

std::string
getRawImageName(){
	return "/image_raw";
}

std::string
getReceiverSensorMsgTopicName(){
	return getTopicName()+"/sensor/rec";
}

std::string
getReceiverImageMsgTopicName(int type){
	return getTopicName() + getCameraName(type) + "/rec";
}

std::string
getSubTopic(std::string topic){
	unsigned long int pos = topic.rfind('/'); // find the last "/"
	return topic.substr(0, pos);
}

std::string
getTopicName(int type){
	return getTopicName() + getCameraName(type);
}

std::string
getTopicNameRawImage(int type){
	return getTopicName(type)+getRawImageName();
}

std::string
getCameraName(int type){
	std::stringstream topic;
	switch (type){
		case ladybug5_network::LADYBUG_RAW_CAM0:
			topic << "/camera0";
			break;
		case ladybug5_network::LADYBUG_RAW_CAM1:
			topic << "/camera1";
			break;
		case ladybug5_network::LADYBUG_RAW_CAM2:
			topic << "/camera2";
			break;
		case ladybug5_network::LADYBUG_RAW_CAM3:
			topic << "/camera3";
			break;
		case ladybug5_network::LADYBUG_RAW_CAM4:
			topic << "/camera4";
			break;
		case ladybug5_network::LADYBUG_RAW_CAM5:
			topic << "/camera5";
			break;
		case ladybug5_network::LADYBUG_PANORAMIC:
			topic << "/panoramic";
			break;
		case ladybug5_network::LADYBUG_SPHERICAL:
			topic << "/spherical";
			break;
		case ladybug5_network::LADYBUG_DOME:
			topic << "/dome";
			break;
		default:
			topic << "/camera_type_" << type;
		}
	return topic.str();
}

std::vector<std::string>
getNodeList()
{
  std::vector<std::string> all_nodes;
  ros::master::getNodes(all_nodes);
//  for (std::vector<std::string>::const_iterator it = all_nodes.begin(); it != all_nodes.end(); it++)
//  {
//	  std::cout << it->data() << std::endl;
//  }
  return all_nodes;
}

std::vector<std::string>
getTopicsOfType(std::string datatype, std::string filter1, std::string endsWith)
{
  //std::cout << "getTopicsOfType: " << datatype << " filter1: " << filter1 << " filter2: " << filter2 << std::endl;
  ros::master::V_TopicInfo topic_info;
  ros::master::getTopics(topic_info);

  std::vector<std::string> all_topics;
  for (ros::master::V_TopicInfo::const_iterator it = topic_info.begin(); it != topic_info.end(); it++)
  {

	   if(it->datatype.compare(datatype) == 0){
		  if(filter1.empty()){
			  all_topics.push_back(it->name);
		  }
		  else {
			  if(it->name.find(filter1.c_str(), 0)!=std::string::npos){
				  if(endsWith.empty()){
					  all_topics.push_back(it->name);
				  }
				  else{
					  if(it->name.find(endsWith.c_str(),it->name.size()-endsWith.size())!=std::string::npos){
						  all_topics.push_back(it->name);
					  }
				  }
			  }
		  }
	  }
  }
  return all_topics;
}

