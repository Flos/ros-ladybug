/*
 * Receivernodelet.cpp
 *
 *  Created on: 27.09.2014
 *      Author: fnolden
 */
#include "receiver_nodelet.h"
#include <pluginlib/class_list_macros.h>

// watch the capitalization carefully
PLUGINLIB_DECLARE_CLASS(ladybug, Receiver_nodelet, ladybug::Receiver_nodelet, nodelet::Nodelet)

namespace ladybug
{
	void
	Receiver_nodelet::onInit() {
		// TODO Auto-generated constructor stub
		name = "Receiver_nodelet";
		sequence = 0;
		nh = getMTPrivateNodeHandle();
		nh.param<std::string>("connection", connection, "tcp://192.168.1.11:28882");


		//nh.param("buffer_size", )
		GOOGLE_PROTOBUF_VERIFY_VERSION;
		sensor_raw_publisher = NULL;
		ROS_INFO_NAMED(name,"pram: connection: %s", connection.c_str());
		thread_ = boost::shared_ptr<boost::thread>(new boost::thread( boost::bind( &Receiver_nodelet::loop, this )));
	}

	void
	Receiver_nodelet::loop(){
		zmq_service.cfg_buffer_recv = 1;
		zmq_service.cfg_linger = 2;
		zmq_service.cfg_request_timeout = 2;
		zmq_service.init(connection, ZMQ_SUB);

		 ladybug5_network::pbMessage message;
		 try{
			while(nh.ok()){
				++sequence;

				if(zmq_service.receive(message))
				{
					ROS_INFO_NAMED(name,"message ok");
					handle_message(message);
					message.Clear();
				}
				else{
					ROS_INFO_NAMED(name,"no message");
					zmq_service.re_init();
				}
				usleep(60);
			}
		}catch (std::exception& e) {
			ROS_ERROR_NAMED(name, "exception in loop: %s ", e.what());
		}


		ROS_DEBUG_NAMED(name, "STOPPING");
	}

	void
	Receiver_nodelet::handle_message(ladybug5_network::pbMessage &recv_msg){
		ROS_INFO_NAMED(name,"handle message start");
		ladybug::sensors sensor_msg;
		sensor_msg.header.frame_id = "ladybug_link";
		sensor_msg.header.seq = sequence;
		sensor_msg.serial_number = recv_msg.serial_number();
		sensor_msg.header.stamp = ros::Time(recv_msg.time().ulseconds(), recv_msg.time().ulmicroseconds()*1000 );
		sensor_msg.accelerometer[0] = recv_msg.sensors().accelerometer().x();
		sensor_msg.accelerometer[1] = recv_msg.sensors().accelerometer().y();
		sensor_msg.accelerometer[2] = recv_msg.sensors().accelerometer().z();

		sensor_msg.gyroscope[0] = recv_msg.sensors().gyroscope().x();
		sensor_msg.gyroscope[1] = recv_msg.sensors().gyroscope().y();
		sensor_msg.gyroscope[2] = recv_msg.sensors().gyroscope().z();

		sensor_msg.compass[0] = recv_msg.sensors().compass().x();
		sensor_msg.compass[1] = recv_msg.sensors().compass().y();
		sensor_msg.compass[2] = recv_msg.sensors().compass().z();

		sensor_msg.temperature = recv_msg.sensors().temperature();
		sensor_msg.humidity = recv_msg.sensors().humidity();
		sensor_msg.barometer = recv_msg.sensors().barometer();

		ROS_INFO_NAMED(name,"image loop");
		/* images */
		for(int i = 0; i< recv_msg.images_size(); ++i){
			if( recv_msg.images(i).has_packages()){ /* packages send also */
				ladybug::imagePtr msg_ptr(new ladybug::image());
				msg_ptr->header.seq = sequence;
				msg_ptr->serial_number = recv_msg.serial_number();
				msg_ptr->width = recv_msg.images(i).width();
				msg_ptr->height = recv_msg.images(i).height();
				msg_ptr->header.stamp =  ros::Time(recv_msg.time().ulseconds(), recv_msg.time().ulmicroseconds()*1000 );
				msg_ptr->image_type = recv_msg.images(i).type();
				msg_ptr->camera = recv_msg.camera();
				msg_ptr->color_encoding = recv_msg.images(i).color_encoding();
				msg_ptr->bayer_encoding = recv_msg.images(i).bayer_encoding();
				msg_ptr->focalX = recv_msg.images(i).distortion().focalx();
				msg_ptr->focalY = recv_msg.images(i).distortion().focaly();
				msg_ptr->centerX = recv_msg.images(i).distortion().centerx();
				msg_ptr->centerY = recv_msg.images(i).distortion().centery();
				msg_ptr->translationX = recv_msg.images(i).position().tx();
				msg_ptr->translationY = recv_msg.images(i).position().ty();
				msg_ptr->translationZ = recv_msg.images(i).position().tz();
				msg_ptr->rotationX = recv_msg.images(i).position().rx();
				msg_ptr->rotationY = recv_msg.images(i).position().ry();
				msg_ptr->rotationZ = recv_msg.images(i).position().rz();

				msg_ptr->border_top = recv_msg.images(i).border_top();
				msg_ptr->border_bottem = recv_msg.images(i).border_bottem();
				msg_ptr->border_left = recv_msg.images(i).border_left();
				msg_ptr->border_right = recv_msg.images(i).border_right();

				switch(recv_msg.images(i).packages()){
					case 1:{
						zmq::message_t rgb;
						if(!zmq_service.receive(rgb, ZMQ_NOBLOCK)){
							printf( "rgb %i\n",i);
							return;
						}

						//ROS_INFO_STREAM_NAMED(NAME, "image " << i << " with size: " << rgb.size() << " Byte, " << rgb.size()/1024 << " KiB");
						msg_ptr->raw = std::vector<uchar>((char*)rgb.data(), (char*)rgb.data()+rgb.size());
						break;
					}
					case 3:{
						zmq::message_t r;
						zmq::message_t g;
						zmq::message_t b;
						if(!zmq_service.receive(r,ZMQ_NOBLOCK)){
							printf( "r %i\n",i);
							return;
						}
						//ROS_INFO_STREAM_NAMED(NAME, "image " << i << " with size: " << r.size() << " Byte, " << r.size()/1024 << " KiB");

						if(!zmq_service.receive(g,ZMQ_NOBLOCK)){
							printf( "g %i\n",i);
							return;
						}
						if(!zmq_service.receive(b,ZMQ_NOBLOCK)){
							printf( "b %i\n",i);
							return;
						}
						msg_ptr->r = std::vector<uchar>((char*)r.data(), (char*)r.data()+r.size());
						msg_ptr->g = std::vector<uchar>((char*)g.data(), (char*)g.data()+g.size());
						msg_ptr->b = std::vector<uchar>((char*)b.data(), (char*)b.data()+b.size());
						break;
					}
					default:
						break;
				}
				publish_image(recv_msg.images(i).type(), msg_ptr);
			}
		}
		if(sensor_raw_publisher == 0){
			sensor_raw_publisher =  new ros::Publisher();
			*sensor_raw_publisher = nh.advertise<ladybug::sensors>(getReceiverSensorMsgTopicName(), 1);
		}
		sensor_raw_publisher->publish(sensor_msg);
		ROS_INFO_NAMED(name,"handle message end");
	}

	void
	Receiver_nodelet::publish_image(int type, ladybug::imagePtr &msg_ptr){
		std::string id = getReceiverImageMsgTopicName(type);
		ros::Publisher* pub = NULL;
		msg_ptr->header.frame_id = getSubTopic(id);
		if (publisher_map.find(id) == publisher_map.end()) {
			//create new publisher
			pub = new ros::Publisher();
			*pub = nh.advertise<ladybug::image>(id, 1);
			publisher_map.insert(std::pair<std::string, ros::Publisher*>(id, pub));
		}
		pub = publisher_map.find(id)->second;
		pub->publish(msg_ptr);
	}

	Receiver_nodelet::~Receiver_nodelet() {
		// TODO Auto-generated destructor stub
		ROS_INFO_NAMED(name,"destructor");
		thread_->join();
		delete sensor_raw_publisher;
		google::protobuf::ShutdownProtobufLibrary();
	}
}
