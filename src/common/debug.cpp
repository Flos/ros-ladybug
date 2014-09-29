/*
 * debug.h
 *
 *  Created on: 29.09.2014
 *      Author: fnolden
 */

#include <debug.h>

void debug_header(std::string name, std_msgs::Header &header){
	ROS_INFO_NAMED(name,"message sequence: %i ROS-time sec: %i nsec: %i Delta: sec: %i nsec: %i",  header.seq, ros::Time().now().sec, ros::Time().now().nsec, ros::Time().now().sec - header.stamp.sec, ros::Time().now().nsec - header.stamp.nsec );
}
