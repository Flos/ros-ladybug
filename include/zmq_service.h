#ifndef ZMQ_SERVICE_H_
#define ZMQ_SERVICE_H_

#include <zmq.hpp>
#include <protobuf/imageMessage.pb.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <string>
#include <stdexcept>

#define REQUEST_TIMEOUT 10 // msecs, (> 1000!)
#define REQUEST_RETRIES 3 // Before we abandon

class Zmq_service
{
public:
	Zmq_service(void);
	std::string connection;
	int type;
	void init(std::string socket, int type);

	bool send(google::protobuf::Message& pb_msg, int flag = 0);
	bool send(ladybug5_network::pb_start_msg& pb_msg, int flag = 0);
	bool send(ladybug5_network::pb_reply& pb_msg, int flag = 0);
	bool send(ladybug5_network::pbMessage& pb_msg, int flag = 0);
	bool send(std::string &string, int flag = 0);
	bool send(zmq::message_t &msg, int flag = 0);
	
	bool receive(google::protobuf::Message& pb_msg, int flag = 0);
	bool receive(ladybug5_network::pb_start_msg& pb_msg, int flag = 0);
	bool receive(ladybug5_network::pb_reply& pb_msg, int flag = 0);
	bool receive(ladybug5_network::pbMessage& pb_msg, int flag = 0);
	bool receive(zmq::message_t &msg, int flag = 0);


	void reset_state();

	~Zmq_service(void);
private:
	void create_socket(int type);
	zmq::context_t*zmq_context; 
	zmq::socket_t* zmq_socket;
	int retries_left;
};
#endif ZMQ_SERVICE_H_

