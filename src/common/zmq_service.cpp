#include "zmq_service.h"


Zmq_service::Zmq_service(void)
{
	cfg_type = 0;
	cfg_connection = "";
	cfg_request_timeout = 10;
	cfg_retries = 3;
	cfg_buffer_recv = 1;
	cfg_buffer_send = 1;
	cfg_linger =  2;
	cfg_force_bind = false;
	retries_left = 0;
	zmq_context = NULL;
	zmq_socket = NULL;
}



// Helper function that returns a new configured socket
// connected to the Hello World server
//
void
Zmq_service::create_socket(int type) {
	printf( "connecting to server…" );
	zmq_socket = new zmq::socket_t(*zmq_context, type);

	// Configure socket to not wait at close time
	if(type == ZMQ_SUB){
		// No filtering for subscribers
		zmq_socket->setsockopt(ZMQ_SUBSCRIBE, 0, 0);
	}
	zmq_socket->setsockopt(ZMQ_LINGER, &cfg_linger, sizeof (cfg_linger));
	zmq_socket->setsockopt(ZMQ_RCVHWM, &cfg_buffer_recv, sizeof(cfg_buffer_recv));  //prevent buffer get overfilled
	zmq_socket->setsockopt(ZMQ_SNDHWM, &cfg_buffer_send, sizeof(cfg_buffer_send));
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
		re_init();
		return false;
	}
	try{
		zmq_socket->send(msg, flag);
		retries_left = cfg_retries;
	}
	catch(std::exception &e){
		re_init();
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
	printf("recv\n");
	bool success = false;
	if(flag == ZMQ_NOBLOCK){
		success = zmq_socket->recv(&msg, flag);
	}
	else
	{
		zmq::pollitem_t items[] = { { *zmq_socket, 0, ZMQ_POLLIN, 0 } };
		zmq::poll (&items[0], 1, cfg_request_timeout * 1000);
		if (items[0].revents & ZMQ_POLLIN){
			printf("recv pollin\n");
			success = zmq_socket->recv(&msg, flag);
		}
		else{
			success = false;
		}
	}
	if(!success && --retries_left == 0){
		re_init();
	}
	if(success){
		retries_left = cfg_retries;
	}
	printf("sucess: %i retries_left: %i\n",success, retries_left);
	return success;
}

void
Zmq_service::init(std::string socket, int type)
{
	printf("init \n");
	retries_left = cfg_retries;

	//ZMQ
	zmq_context = new zmq::context_t(1);
	this->cfg_type = type;
	this->cfg_connection = socket;

	create_socket(cfg_type);

    if(zmq_socket == NULL || zmq_context == NULL){
    	throw new std::runtime_error("Cannot create zmq socket");
    }

    if(cfg_force_bind){
    	zmq_socket->bind(cfg_connection.c_str());
    }
    else{
		switch (cfg_type){
			case ZMQ_REQ:
			case ZMQ_PULL:
			case ZMQ_SUB:
				printf("connect socket: %s\n", cfg_connection.c_str());
				zmq_socket->connect(cfg_connection.c_str());
				break;
			default:
				printf("bind socket: %s\n", cfg_connection.c_str());
				zmq_socket->bind(cfg_connection.c_str());
				break;
		}
    }
}

void
Zmq_service::reset_state(){

	printf( "Reset " );
	zmq_socket->disconnect(cfg_connection.c_str());
	zmq_socket->close();
	delete zmq_socket;
	delete zmq_context;
	zmq_socket = NULL;
	zmq_context = NULL;
}

void
Zmq_service::re_init(){
	//printf( "reinit" );
	reset_state();
	init(cfg_connection, cfg_type);
}

Zmq_service::~Zmq_service(){
	zmq_socket->close();
	delete zmq_socket;
	delete zmq_context;
}
