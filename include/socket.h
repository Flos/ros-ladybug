/*
 * socket.h
 *
 *  Created on: 05.12.2013
 *      Author: florian
 */

#ifndef SOCKET_H_
#define SOCKET_H_

#include <sstream>
#include "protobuf/imageMessage.pb.h"
#include "google/protobuf/io/coded_stream.h"
#include "google/protobuf/io/zero_copy_stream_impl.h"
#include "zmq.hpp"
#include "time.h"

ladybug5_network::pbMessage*
socket_read(zmq::socket_t* socket);

ladybug5_network::pbImage*
socket_read_pbImage(zmq::socket_t* socket);

void
socket_write(zmq::socket_t* socket, ladybug5_network::pbMessage*, int flag = 0);

void
socket_write_pbImage(zmq::socket_t* socket, ladybug5_network::pbImage*, int flag = 0);

#endif /* SOCKET_H_ */
