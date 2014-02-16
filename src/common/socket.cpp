#include "socket.h"
#include "assert.h"

ladybug5_network::pbMessage*
socket_read(zmq::socket_t* socket){
  zmq::message_t msg_recieved;
  socket->recv(&msg_recieved);

  ladybug5_network::pbMessage* pb_message = new ladybug5_network::pbMessage();
  pb_message->ParseFromArray(msg_recieved.data(), msg_recieved.size() );
  return pb_message;
}

void
socket_write(zmq::socket_t* socket, ladybug5_network::pbMessage* message, int flag){
  // Socket to talk to clients
    zmq::message_t request( message->ByteSize());
    message->SerializeToArray(request.data(), message->ByteSize());
    socket->send(request, flag);
}

ladybug5_network::pbImage*
socket_read_pbImage(zmq::socket_t* socket){
	zmq::message_t msg_recieved;
	socket->recv(&msg_recieved);

	ladybug5_network::pbImage* pb_image = new ladybug5_network::pbImage();
	pb_image->ParseFromArray(msg_recieved.data(), msg_recieved.size() );
	return pb_image;
}

void
socket_write_pbImage(zmq::socket_t* socket, ladybug5_network::pbImage* pbImage, int flag){
	zmq::message_t request( pbImage->ByteSize());
	pbImage->SerializeToArray(request.data(), pbImage->ByteSize());
	socket->send(request, flag);
}
