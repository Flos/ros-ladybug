/*
 * debug.h
 *
 *  Created on: 29.09.2014
 *      Author: fnolden
 */

#ifndef DEBUG_H_
#define DEBUG_H_

#include <ros/console.h>
#include "std_msgs/Header.h"

void debug_header(std::string name, std_msgs::Header &header);

#endif /* DEBUG_H_ */
