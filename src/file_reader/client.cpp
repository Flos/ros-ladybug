#include "socket.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <ros/console.h>

#define NAME "image_publisher_test_client"
#define CONNECTION "tcp://127.0.0.1:28882"
#define PATHIMG1 "/home/fnolden/Bilder/input1.big.jpg"
#define PATHIMG2 "/home/fnolden/Bilder/input2.big.jpg"
#define USLEEP 1000000/10/6 //Images per second
#define LOOP 1000000
#define NAME_CAMERA "filestream"
#define NAME_IMAGE "input"
#define NUMBER_IMAGES 1

void
FillMessage(ladybug5_network::pbMessage& message, std::string path){
  message.set_id(2);
  message.set_name(NAME);
  message.set_camera(NAME_CAMERA);
  ladybug5_network::pbImage* image = 0;
  image = message.add_images();
  int nr = message.images_size();
  image->set_number(nr);
  image->set_type(ladybug5_network::LADYBUG_RAW_CAM0);
  std::stringstream name;
  name << NAME_IMAGE << nr;
  image->set_name(name.str());

  int size;
  char* memblock;
  ROS_INFO_STREAM_NAMED(NAME, "Starting to load the file: " << path );

   std::ifstream file(path.c_str(), std::ios::binary);
   if (file.is_open())
   {
     file.seekg(0, file.end);
     size = file.tellg();
     assert(size != 0);
     memblock = new char [size];
     file.seekg(0, file.beg);
     file.read(memblock, size);
     file.close();
	 image->set_image(memblock, size);
	 image->set_size(size);

     ROS_INFO_STREAM_NAMED(NAME, "Filesize: " << (double) size /(1024) << " KiB Imagesize: " << (double) image->size() / (1024));
   }
   delete[] memblock;
}

int main(int argc, char **argv)
{
  std::string connection = CONNECTION;
  std::string pathImage1 = PATHIMG1;
  std::string pathImage2 = PATHIMG2;

  GOOGLE_PROTOBUF_VERIFY_VERSION;

  zmq::context_t ctx(1);
  zmq::socket_t socket(ctx, ZMQ_PUB);

  int one = 1;
  // Prevent publisher overflow from slow subscribers
  socket.setsockopt(ZMQ_RCVHWM, &one, sizeof(one));
  socket.setsockopt(ZMQ_SNDHWM, &one, sizeof(one));

  socket.connect(connection.c_str());

  ROS_INFO_STREAM_NAMED(NAME, "Connecting to " << connection );
  ladybug5_network::pbMessage message1;
  FillMessage(message1, pathImage1.c_str());
  ladybug5_network::pbMessage message2;
  FillMessage(message2, pathImage2.c_str());


  for (int i = 1;  i < NUMBER_IMAGES; ++i ) {
	  FillMessage(message1, pathImage1.c_str());
	  FillMessage(message2, pathImage2.c_str());
  }


  ladybug5_network::pbMessage* current;

  for(int i=0 ; i < LOOP; ++i){
	  if( socket.connected() ){
		if(i%2==0){
			current = &message1;
		}
		else{
			current = &message2;
		}
		current->set_id(i);
		socket_write(&socket, current);
		ROS_INFO_STREAM_NAMED(NAME, "Send image: " << i%2 << " size: " << current->ByteSize()/(1024) << " KiB " << i);
	  }
	  else{
		ROS_INFO_STREAM_NAMED(NAME, "Not connected");
	  }
    usleep(USLEEP);
  }

  google::protobuf::ShutdownProtobufLibrary();
  return 0;
}
