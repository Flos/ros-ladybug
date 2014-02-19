/*
 * helper.cpp
 *
 *  Created on: 13.02.2014
 *      Author: fnolden
 */

#include "helper.h"

std::string getImageId(ladybug5_network::pbMessage* message, int i){
	return  "/" + message->name() + "/" + message->camera()	+ "/" + message->images(i).name();
}

std::string getSensorTopic(ladybug5_network::pbMessage* message){
	return  "/" + message->name() + "/" + message->camera()	+ "/sensors" ;
}

std::string getImageIdColorSep(ladybug5_network::pbMessage* message, int i){
	return  getImageId(message, i) + "/" + "ladybug_image";
}

std::string getCompressedMessageTopic(ladybug5_network::pbMessage* message){
	return  "/" + message->name() + "/" + message->camera() + "/" + "compressed";
}

std::string make_daytime_string(){
  using namespace std; // For time_t, time and ctime;
  time_t now = time(0);
  return ctime(&now);
}
