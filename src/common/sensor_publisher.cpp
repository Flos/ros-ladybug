/*
 * colorSepToImage.cpp
 *
 *  Created on: 15.02.2014
 *      Author: fnolden
 */

#include "sensor_publisher.h"

sensor_publisher::sensor_publisher(std::string subscribe_topic) {
	//Topic to subscribe
	hasPublisher = false;
	subscribe_topic_ = subscribe_topic;

	ROS_INFO_STREAM("Creating processing node for topic" << subscribe_topic.c_str());
	sub_ = n_.subscribe(subscribe_topic.c_str(), 1, &sensor_publisher::callback, this);
}

sensor_publisher::~sensor_publisher() {
	ROS_INFO_STREAM("closing node for topic" << subscribe_topic_);
}

void
sensor_publisher::callback(const ladybug::sensors &input)
{
	if(publish_topic_.empty()){

		ROS_INFO("publisher created");
		//Topic to publish
		publish_topic_ = getTopicName()+"/sensors";
		pub_humidity_ = n_.advertise<sensor_msgs::RelativeHumidity>(publish_topic_ + "relative_humidity", 1);
		pub_temperature_ = n_.advertise<sensor_msgs::Temperature>(publish_topic_ + "temperature", 1);
		pub_imu_ = n_.advertise<sensor_msgs::Imu>(publish_topic_ + "imu", 1);
		hasPublisher = true;
	}
	else{ /* not looking right */
		//ROS_INFO("Has publisher");
		humidity_msg.header = input.header;
		humidity_msg.relative_humidity = input.humidity;
		pub_humidity_.publish(humidity_msg);

		temperatur_msg.header = input.header;
		temperatur_msg.temperature = ((double)input.temperature)-273.15; // °Kalvin to °Celsius
		pub_temperature_.publish(temperatur_msg);

		imu_msg.header = input.header;
		imu_msg.linear_acceleration.x = input.accelerometer[0];
		imu_msg.linear_acceleration.y = input.accelerometer[1];
		imu_msg.linear_acceleration.z = input.accelerometer[2];
		imu_msg.angular_velocity.x = input.gyroscope[0];
		imu_msg.angular_velocity.y = input.gyroscope[1];
		imu_msg.angular_velocity.z = input.gyroscope[2];
		tf::Quaternion quat;
		quat.setRPY(input.compass[2], input.compass[1], input.compass[0]);
		imu_msg.orientation.x = quat.getX();
		imu_msg.orientation.y = quat.getY();
		imu_msg.orientation.z = quat.getZ();
		imu_msg.orientation.w = quat.getW();
		pub_imu_.publish(imu_msg);
	}
}

