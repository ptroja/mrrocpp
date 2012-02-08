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
	int pid = (int) getpid();

	char hostname[256];

	if(gethostname(hostname, 256) == -1) {
		throw std::runtime_error("Could not get hostname");
	}

	std::stringstream ss;

	ss << "R:" << name << "@" << hostname << ":" << port << "/" << pid ;

	zmq::message_t request(ss.str().size());

	memcpy(request.data(), ss.str().c_str(), ss.str().size());

	sock.send(request);

	zmq::message_t reply;

	sock.recv(&reply);
}

void locator::unregister_name(const std::string & name)
{
	std::stringstream ss;

	ss << "U:" << name;

	zmq::message_t request(ss.str().size());

	memcpy(request.data(), ss.str().c_str(), ss.str().size());

	sock.send(request);

	zmq::message_t reply;

	sock.recv(&reply);
}

std::string locator::locate_name(const std::string & name)
{
	std::stringstream ss;

	ss << "?:" << name;

	zmq::message_t request(ss.str().size());

	memcpy(request.data(), ss.str().c_str(), ss.str().size());

	sock.send(request);

	zmq::message_t reply;

	sock.recv(&reply);

	return std::string((char *) reply.data());
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

int bind_ephemeral(zmq::socket_t & sock)
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
	: my_name(name), sock(context::instance().get(), ZMQ_SUB)
{
	port = bind_ephemeral(sock);

	locator::instance().register_name(my_name, port);
}

sender::sender(const std::string & name)
	: remote_name(name), sock(context::instance().get(), ZMQ_PUB)
{
	locator::instance().locate_name(remote_name);

	bind_ephemeral(sock);
}

}

#endif /* ZMQPP_CC_ */
