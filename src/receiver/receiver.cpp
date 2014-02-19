#include "ros/ros.h"
#include "ladybug/image.h"
#include "ladybug/sensors.h"
#include <sstream>
#include "socket.h"
#include "opencv_helper.h"
#include <string.h>
#include <map>
#include <iostream>
#include "helper.h"
#include <sensor_msgs/image_encodings.h>

#define NAME "receiver"
#define ZMQ_BUFFER_SIZE 20

int main(int argc, char **argv){
  std::string connection = "tcp://*:28882";
  int zmqBufferSize = ZMQ_BUFFER_SIZE;

  ROS_INFO_STREAM_NAMED(NAME, "Starting publisher, listening on " << connection.c_str());
  ROS_INFO_STREAM_NAMED(NAME, "Maximum Messages in Buffer "<< zmqBufferSize);

  GOOGLE_PROTOBUF_VERIFY_VERSION;

  ros::init(argc, argv, NAME);
  ros::NodeHandle nh;

  std::map<std::string, ros::Publisher*> publisher_map;
  unsigned int sequence = 0;

  try{
      zmq::context_t context(1);
      zmq::socket_t socket(context, ZMQ_SUB );

      socket.setsockopt(ZMQ_SUBSCRIBE, 0, 0);
      socket.setsockopt(ZMQ_RCVHWM, &zmqBufferSize, sizeof(zmqBufferSize));
      socket.setsockopt(ZMQ_SNDHWM, &zmqBufferSize, sizeof(zmqBufferSize));
      socket.bind(connection.c_str());
      ladybug5_network::pbMessage* message;

      while(nh.ok()){
		message = socket_read(&socket);
		ROS_INFO_STREAM_NAMED(NAME, "Received message id: " << message->id() << " with size: " << message->ByteSize()/1024 << " KiB and " << message->images_size() << " images");
		/* Sensor messages */
		std::string sensorTopic = getSensorTopic(message);
		if (publisher_map.find(sensorTopic) == publisher_map.end()) {
			//create new publisher
			ros::Publisher* pub =  new ros::Publisher();
			*pub = nh.advertise<ladybug::sensors>(sensorTopic, 5);
			publisher_map.insert(std::pair<std::string, ros::Publisher*>(sensorTopic, pub));
		}
		ros::Publisher* sensorPublisher = publisher_map.find(sensorTopic)->second;
		ladybug::sensors sensor_msg;
		sensor_msg.header.frame_id = sensorTopic;
		sensor_msg.header.seq = sequence;
		sensor_msg.serial_number = message->serial_number();
		sensor_msg.header.stamp = ros::Time(message->time().ulseconds(), message->time().ulmicroseconds()*1000 );
		sensor_msg.accelerometer[0] = message->sensors().accelerometer().x();
		sensor_msg.accelerometer[1] = message->sensors().accelerometer().y();
		sensor_msg.accelerometer[2] = message->sensors().accelerometer().z();

		sensor_msg.gyroscope[0] = message->sensors().gyroscope().x();
		sensor_msg.gyroscope[1] = message->sensors().gyroscope().y();
		sensor_msg.gyroscope[2] = message->sensors().gyroscope().z();

		sensor_msg.compass[0] = message->sensors().compass().x();
		sensor_msg.compass[1] = message->sensors().compass().y();
		sensor_msg.compass[2] = message->sensors().compass().z();

		sensor_msg.temperature = message->sensors().temperature();
		sensor_msg.humidity = message->sensors().humidity();
		sensor_msg.barometer = message->sensors().barometer();

		/* images */
		for(int i = 0; i< message->images_size(); ++i){
			std::string id = getImageIdColorSep(message, i);
			if (publisher_map.find(id) == publisher_map.end()) {
				//create new publisher
				ros::Publisher* pub =  new ros::Publisher();
				*pub = nh.advertise<ladybug::image>(id, 5);
				publisher_map.insert(std::pair<std::string, ros::Publisher*>(id, pub));
			}
			ros::Publisher* pub = publisher_map.find(id)->second;

			if( message->images(i).has_packages()){ /* packages send also */
				ladybug::image msg;
				msg.header.frame_id = id;
				msg.header.seq = sequence;
				msg.serial_number = message->serial_number();
				msg.width = message->images(i).width();
				msg.height = message->images(i).height();
				msg.header.stamp =  ros::Time(message->time().ulseconds(), message->time().ulmicroseconds()*1000 );

				msg.camera_number = i;
				msg.camera = message->camera();
				msg.color_encoding = message->images(i).color_encoding();
				msg.bayer_encoding = message->images(i).bayer_encoding();
				msg.focalX = message->images(i).distortion().focalx();
				msg.focalY = message->images(i).distortion().focaly();
				msg.centerX = message->images(i).distortion().centerx();
				msg.centerY = message->images(i).distortion().centery();
				msg.translation[0] = message->images(i).position().tx();
				msg.translation[1] = message->images(i).position().ty();
				msg.translation[2] = message->images(i).position().tz();
				msg.rotation[0] = message->images(i).position().rx();
				msg.rotation[1] = message->images(i).position().ry();
				msg.rotation[2] = message->images(i).position().rz();

				switch(message->images(i).packages()){
					case 1:{
						zmq::message_t rgb;
						socket.recv(&rgb);
						msg.raw = std::vector<uchar>((char*)rgb.data(), (char*)rgb.data()+rgb.size());
						break;
					}
					case 3:{
						zmq::message_t r;
						zmq::message_t g;
						zmq::message_t b;
						socket.recv(&r);
						socket.recv(&g);
						socket.recv(&b);
						msg.r = std::vector<uchar>((char*)r.data(), (char*)r.data()+r.size());
						msg.g = std::vector<uchar>((char*)g.data(), (char*)g.data()+g.size());
						msg.b = std::vector<uchar>((char*)b.data(), (char*)b.data()+b.size());
						break;
					}
					default:
						break;
				}
				pub->publish(msg);
			}
		}
		sensorPublisher->publish(sensor_msg);

		message->Clear();
		delete message;
		++sequence;
		ros::spinOnce();
      }
  }catch (std::exception& e) {
      std::cerr << e.what() << std::endl;
  }

  google::protobuf::ShutdownProtobufLibrary();
  ROS_INFO_STREAM_NAMED(NAME, "STOPPING");

  return 0;
}
