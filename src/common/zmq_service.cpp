#include "zmq_service.h"


Zmq_service::Zmq_service(void)
{
	type = 0;
	connection = "";
	zmq_socket = NULL;
	retries_left = 0;
	zmq_context = new zmq::context_t();
}



// Helper function that returns a new configured socket
// connected to the Hello World server
//
void
Zmq_service::create_socket(int type) {
	//printf( "connecting to server…" );
	zmq_socket = new zmq::socket_t(*zmq_context, type);

	// Configure socket to not wait at close time
	int linger = 2;
	int val = 2;
	zmq_socket->setsockopt(ZMQ_LINGER, &linger, sizeof (linger));
	zmq_socket->setsockopt(ZMQ_RCVHWM, &val, sizeof(val));  //prevent buffer get overfilled
	zmq_socket->setsockopt(ZMQ_SNDHWM, &val, sizeof(val));
}

bool 
Zmq_service::send(google::protobuf::Message& pb_msg, int flag){
	std::string pb_serialized;
	pb_msg.SerializeToString(&pb_serialized);

	// create and send the zmq message
	return send(pb_serialized, flag);
}

bool 
Zmq_service::send(ladybug5_network::pb_start_msg& pb_msg, int flag){
	std::string pb_serialized;
	pb_msg.SerializeToString(&pb_serialized);

	// create and send the zmq message
	return send(pb_serialized, flag);
}

bool 
Zmq_service::send(ladybug5_network::pb_reply& pb_msg, int flag){
	std::string pb_serialized;
	pb_msg.SerializeToString(&pb_serialized);

	// create and send the zmq message
	return send(pb_serialized, flag);
}

bool 
Zmq_service::send(ladybug5_network::pbMessage& pb_msg, int flag){
	std::string pb_serialized;
	pb_msg.SerializeToString(&pb_serialized);

	// create and send the zmq message
	return send(pb_serialized, flag);
}

bool 
Zmq_service::send(std::string &string, int flag){
	zmq::message_t msg (string.size());
	memcpy ((void *) msg.data(), string.c_str(), string.size());

	return send(msg, flag);
}

bool 
Zmq_service::send(zmq::message_t &msg, int flag){
	if(!zmq_socket->connected() || --retries_left == 0 ){
		reset_state();
		init(connection, type);
		//printf( "resetting send " );
		return false;
	}
	try{
		zmq_socket->send(msg, flag);
		retries_left = REQUEST_RETRIES;
	}
	catch(std::exception &e){
		reset_state();
		init(connection, type);
		return false;
	}
	return true;
}

bool 
Zmq_service::receive(google::protobuf::Message& pb_msg, int flag){
	zmq::message_t msg;
	
	if(!receive(msg, flag)) return false;
	pb_msg.ParseFromArray(msg.data(), msg.size());
	return true;
}

bool 
Zmq_service::receive(ladybug5_network::pb_start_msg& pb_msg, int flag){
	zmq::message_t msg;
	
	if(!receive(msg, flag)) return false;
	pb_msg.ParseFromArray(msg.data(), msg.size());
	return true;
}

bool 
Zmq_service::receive(ladybug5_network::pb_reply& pb_msg, int flag){
	zmq::message_t msg;
	
	if(!receive(msg, flag)) return false;
	pb_msg.ParseFromArray(msg.data(), msg.size());
	return true;
}

bool 
Zmq_service::receive(ladybug5_network::pbMessage& pb_msg, int flag){
	zmq::message_t msg;
	
	if(!receive(msg, flag)) return false;
	pb_msg.ParseFromArray(msg.data(), msg.size());
	return true;
}

bool 
Zmq_service::receive(zmq::message_t &msg, int flag){
	//printf("recv\n");
	zmq::pollitem_t items[] = { { *zmq_socket, 0, ZMQ_POLLIN, 0 } };
	zmq::poll (&items[0], 1, REQUEST_TIMEOUT * 1000);

	try{
	// If we got a reply, process it
		if (items[0].revents & ZMQ_POLLIN){
			//printf("recv pollin\n");
			return zmq_socket->recv(&msg, flag);
		}
		else if (flag != ZMQ_NOBLOCK && --retries_left == 0){ //reset socket
				//printf("resetting recv \n");
				reset_state();
				init(connection,type);
				return false;
		}
	}catch(std::exception &e){
		reset_state();
		init(connection,type);
	}
	return false;
}

void
Zmq_service::init(std::string socket, int type)
{
	printf("init \n");
	int retries_left = REQUEST_RETRIES;

	//ZMQ
	this->type = type;
	this->connection = socket;

	create_socket(type);

    if(zmq_socket == NULL || zmq_context == NULL){
    	throw new std::runtime_error("Cannot create zmq socket");
    }

	printf("socket: %s\n", socket.c_str());
	switch (type){
		case ZMQ_REQ:
		case ZMQ_PUSH:
			zmq_socket->connect(socket.c_str());
			break;
		default:
			zmq_socket->bind(socket.c_str());
			break;
	}
}

void
Zmq_service::reset_state(){

	printf( "Reset " );
	zmq_socket->disconnect(connection.c_str());
	delete zmq_socket;
}

Zmq_service::~Zmq_service(){
	zmq_socket->close();
	delete zmq_socket;
	delete zmq_context;
}
