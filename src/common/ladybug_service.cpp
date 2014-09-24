#include "ladybug_service.h"

Ladybug_service::Ladybug_service(){
	nh = NULL;

}

void Ladybug_service::init(ros::NodeHandle &nh){
	this->nh = &nh;
	nh.param<std::string>("connection",connection,"tcp://192.168.1.11:28883");
	ros_service = nh.advertiseService(name, &Ladybug_service::callback_srv, this);
	ROS_INFO_NAMED(name, "ns: %s, name: %s connecting to: %s",nh.getNamespace().c_str(), name, connection.c_str());
	zmq_service.init(connection, ZMQ_REQ);
}

bool Ladybug_service::callback_srv(ladybug::send_command::Request &req, ladybug::send_command::Response &res){
	  ladybug5_network::pb_start_msg msg_req;
	  ladybug5_network::pb_reply msg_reply;

	  msg_req.set_name(req.command.c_str());
	  //ROS_INFO_NAMED(node_name,"Sending: %s",msg_req.DebugString().c_str());

	  try{
		  if (zmq_service.send(msg_req) == true){
			  if(zmq_service.receive(msg_reply) == true){
				  //ROS_INFO_NAMED(node_name,"got response: %s", msg_reply.DebugString().c_str());
				  res.success = msg_reply.success();
				  res.message = msg_reply.info();

				  if(!res.success){
						  res.message += "\nNo path for key: \"" + req.command
								  + "\" available on \"" + connection + "\"\n"
								  +"see config above for available keys.\n";
				  }
			  }
			  else{
				  res.success = false;
				  res.message = "No response from " + connection;
				  ROS_INFO_NAMED(name, res.message.c_str());
			  }
		  }
		  else{
			  res.success = false;
			  res.message = "No response from " + connection;
			  ROS_INFO_NAMED(name, res.message.c_str());
		  }
	  }
	  catch(std::exception &e){
		  ROS_ERROR_NAMED(name, "Exception: %s", e.what());
		  res.success = false;
		  return false;
	  }
	  msg_reply.Clear();
	  msg_req.Clear();

	  return true;
}
