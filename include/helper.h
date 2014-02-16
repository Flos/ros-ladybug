/*
 * helper.h
 *
 *  Created on: 13.02.2014
 *      Author: fnolden
 */

#ifndef HELPER_H_
#define HELPER_H_

#include <string>
#include "protobuf/imageMessage.pb.h"

std::string getImageId(ladybug5_network::pbMessage* message, int i);
std::string getImageIdColorSep(ladybug5_network::pbMessage* message, int i);
std::string getCompressedMessageTopic(ladybug5_network::pbMessage* message);
std::string make_daytime_string();

#endif /* HELPER_H_ */
