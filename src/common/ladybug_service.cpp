#include "ladybug_service.h"

Ladybug_service::Ladybug_service(){
	nh = NULL;

}

void Ladybug_service::init(ros::NodeHandle &nh){
	this->nh = &nh;
	nh.param<std::string>("connection",connection,"tcp://192.168.56.1:28883");
	nh.param<std::string>("name", node_name, "ladybug_service" );
	ros_service = nh.advertiseService(node_name, &Ladybug_service::callback_srv, this);
	zmq_service.init(connection, ZMQ_REQ);
}

bool Ladybug_service::callback_srv(ladybug::send_command::Request &req, ladybug::send_command::Response &res){
	  ladybug5_network::pb_start_msg msg_req;
	  ladybug5_network::pb_reply msg_reply;

	  msg_req.set_name(req.command.c_str());
	  //ROS_INFO_NAMED(node_name,"Sending: %s",msg_req.DebugString().c_str());

	  try{
		  if (zmq_service.send(msg_req) == true && zmq_service.receive(msg_reply) == true){
			  //ROS_INFO_NAMED(node_name,"got response: %s", msg_reply.DebugString().c_str());
			  res.success = msg_reply.success();
			  res.message = msg_reply.info();

			  if(!res.success){
					  res.message += "\nNo path for key: \"" + req.command
							  + "\" available on \"" + connection + "\"\n"
							  +"see config above for available keys.\n";
			  }
		  }
		  else {
			  ROS_INFO_NAMED(node_name,"no response");
			  res.success = false;
			  res.message = "No response from " + connection;
		  }
	  }catch(std::exception &e){
		  ROS_ERROR_NAMED(node_name, "Exception: %s", e.what());
		  res.success = false;
		  return false;
	  }
	  msg_reply.Clear();
	  msg_req.Clear();

	  return true;
}
