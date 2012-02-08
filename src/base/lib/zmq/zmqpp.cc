/*
 * zmqpp.cc
 *
 *  Created on: Feb 7, 2012
 *      Author: ptroja
 */

#ifndef ZMQPP_CC_
#define ZMQPP_CC_

#include <sstream>
#include <stdexcept>

#include <stdio.h>
#include <unistd.h>

#include <zmq.hpp>

#include "zmqpp.h"

namespace zmqpp {

boost::once_flag context::once = BOOST_ONCE_INIT;

context * context::global = NULL;

context & context::instance()
{
	boost::call_once(&create, once);

	return *global;
}

void context::create()
{
	global = new context();
}

context::context()
	: cnx(1)
{
}

zmq::context_t & context::get()
{
	return cnx;
}

locator & locator::instance()
{
	boost::call_once(&create, once);

	return *global;
}

boost::once_flag locator::once = BOOST_ONCE_INIT;

locator * locator::global = NULL;

void locator::create()
{
	global = new locator();
}

locator::locator()
	: sock(context::instance().get(), ZMQ_REQ)
{
	sock.connect("tcp://localhost:5555");
}

void locator::register_name(const std::string & name, int port)
{
	location me;

	char hostname[256];

	if(gethostname(hostname, 256) == -1) {
		throw std::runtime_error("Could not get hostname");
	}

	me.type = location::REGISTER;
	me.host = hostname;
	me.name = name;
	me.pid = (int) getpid();
	me.port = port;

	send(sock, me);

	recv(sock, me);

	if(me.type != location::ACK) {
		throw std::runtime_error("Failed to register name");
	}
}

void locator::unregister_name(const std::string & name)
{
	location me;

	me.type = location::UNREGISTER;
	me.name = name;

	send(sock, me);

	recv(sock, me);

	if(me.type != location::ACK) {
		throw std::runtime_error("Failed to unregister name");
	}
}

location locator::locate_name(const std::string & name)
{
	location target;

	target.type = location::QUERY;
	target.name = name;

	send(sock, target);

	recv(sock, target);

	if(target.type != location::ACK) {
		throw std::runtime_error("Failed to locate name");
	}

	return target;
}

//  This port range is defined by IANA for dynamic or private ports
//  We use this when choosing a port for dynamic binding.
#define ZSOCKET_DYNFROM     0xc000
#define ZSOCKET_DYNTO       0xffff

//  --------------------------------------------------------------------------
//  Bind a socket to a formatted endpoint. If the port is specified as
//  '*', binds to any free port from ZSOCKET_DYNFROM to ZSOCKET_DYNTO
//  and returns the actual port number used. Otherwise asserts that the
//  bind succeeded with the specified port number. Always returns the
//  port number if successful.

int bind_ephemeral_tcp(zmq::socket_t & sock)
{
	//  Ephemeral port needs 4 additional characters
	char endpoint [256 + 4];

	int port, rc = -1;

	for (port = ZSOCKET_DYNFROM; port < ZSOCKET_DYNTO; port++) {
		sprintf(endpoint, "tcp://*:%d", port);
		if (zmq_bind(sock, endpoint) == 0) {
			rc = port;
			break;
		}
	}

	return port;
}

receiver::receiver(const std::string & name)
	: my_name(name),
	  tcp_sock(context::instance().get(), ZMQ_SUB),
	  ipc_sock(context::instance().get(), ZMQ_SUB),
	  inproc_sock(context::instance().get(), ZMQ_SUB)
{
	port = bind_ephemeral_tcp(tcp_sock);

	locator::instance().register_name(my_name, port);
}

receiver::~receiver()
{
	locator::instance().unregister_name(my_name);
}

sender::sender(const std::string & name)
	: remote_name(name), sock(context::instance().get(), ZMQ_PUB)
{
	// Get own location.
	char hostname[256];

	if(gethostname(hostname, 256) == -1) {
		throw std::runtime_error("Could not get hostname");
	}

	location remote = locator::instance().locate_name(remote_name);

	// Build address with string stream.
	std::ostringstream os;

	// Check if we are on the same machine.
	if(remote.host == hostname) {
		// Check if we are within the same process.
		if(remote.pid == (int) getpid()) {
			// Connect with INPROC transport.
			os << "inproc://" << remote_name;
		} else {
			// Connect with IPC transport.
			os << "ipc://tmp/" << remote.pid << "/" << remote_name;
		}
	} else {
		// Connect with TCP transport.
		os << "tcp://" << remote.host << ":" << remote.port;
	}

	// Connect.
	sock.connect(os.str().c_str());
}

}

#endif /* ZMQPP_CC_ */
