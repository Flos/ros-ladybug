#include "ros/ros.h"
#include "ladybug/image_colorsep.h"
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

		for(int i = 0; i< message->images_size(); ++i){
			std::string id = getImageIdColorSep(message, i);
			if (publisher_map.find(id) == publisher_map.end()) {
				//create new publisher
				ros::Publisher* pub =  new ros::Publisher();
				*pub = nh.advertise<ladybug::image_colorsep>(id, 5);
				publisher_map.insert(std::pair<std::string, ros::Publisher*>(id, pub));
			}
			ros::Publisher* pub = publisher_map.find(id)->second;

			if( message->images(i).has_packages()){ /* packages send also */
				ladybug::image_colorsep msg;

				zmq::message_t r;
				zmq::message_t g;
				zmq::message_t b;
				socket.recv(&r);
				socket.recv(&g);
				socket.recv(&b);

				msg.r = std::vector<uchar>((char*)r.data(), (char*)r.data()+r.size());
				msg.g = std::vector<uchar>((char*)g.data(), (char*)g.data()+g.size());
				msg.b = std::vector<uchar>((char*)b.data(), (char*)b.data()+b.size());

				msg.header.frame_id = id;
				msg.header.seq = sequence;
				msg.width = message->images(i).width();
				msg.height = message->images(i).height();
				if(message->images(i).has_time())
				{
					msg.header.stamp =  ros::Time(message->images(i).time().ulseconds(), message->images(i).time().ulmicroseconds()*1000 );
				}
				msg.camera_number = i;
				msg.camera = message->camera();
				msg.encoding = "jpg_color_sep";
				pub->publish(msg);
			}
		}

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
