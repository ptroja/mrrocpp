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

#include <boost/bind.hpp>
#include <boost/thread/thread_time.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp>

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

	{
		boost::mutex::scoped_lock lock(mtx);

		send(sock, me);

		recv(sock, me);
	}

	if(me.type != location::ACK) {
		throw std::runtime_error("Failed to register name");
	}
}

void locator::unregister_name(const std::string & name)
{
	location me;

	me.type = location::UNREGISTER;
	me.name = name;

	{
		boost::mutex::scoped_lock lock(mtx);

		send(sock, me);

		recv(sock, me);
	}

	if(me.type != location::ACK) {
		throw std::runtime_error("Failed to unregister name");
	}
}

location locator::locate_name(const std::string & name)
{
	location target;

	target.type = location::QUERY;
	target.name = name;

	{
		boost::mutex::scoped_lock lock(mtx);

		send(sock, target);

		recv(sock, target);
	}

	if(target.type != location::ACK) {
		throw std::runtime_error("Failed to locate name");
	}

	return target;
}

void locator::ping(const std::string & name)
{
	// Setup ping message.
	location ping_message;

	ping_message.type = location::PING;
	ping_message.name = name;

	{
		boost::mutex::scoped_lock lock(mtx);

		send(sock, ping_message);

		recv(sock, ping_message);
	}
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

publisher::publisher(const std::string & name)
	: my_name(name),
	  tcp_sock(context::instance().get(), ZMQ_PUB),
	  ipc_sock(context::instance().get(), ZMQ_PUB),
	  inproc_sock(context::instance().get(), ZMQ_PUB)
{
	// Bind to TCP socket.
	port = bind_ephemeral_tcp(tcp_sock);

	// Register name.
	locator::instance().register_name(my_name, port);

	// High-water mark setting.
	const uint64_t hwm_value = 1;

	// Set high-water mark action.
	tcp_sock.setsockopt(ZMQ_HWM, &hwm_value, sizeof(hwm_value));
	ipc_sock.setsockopt(ZMQ_HWM, &hwm_value, sizeof(hwm_value));
	inproc_sock.setsockopt(ZMQ_HWM, &hwm_value, sizeof(hwm_value));

	// Spawn keep-alive thread.
	tid = boost::thread(boost::bind(&publisher::ping, this));
}

publisher::~publisher()
{
	// Terminate keep-alive thread.
	tid.interrupt();

	// Join keep-alive thread.
	tid.join();

	// Unregister name.
	locator::instance().unregister_name(my_name);
}

void publisher::ping()
{
	// Setup ping interval.
	const boost::posix_time::time_duration ping_interval = boost::posix_time::milliseconds(250);

	// Setup wakup timer.
	boost::system_time wakeup = boost::get_system_time();

	while(true) {
		// Increment wakeup timer.
		wakeup += ping_interval;

		// Sleep (note: this is also thread interruption point).
		boost::thread::sleep(wakeup);

		// Ping.
		locator::instance().ping(my_name);
	}
}

subscriber::subscriber(const std::string & remote_name_)
	: remote_name(remote_name_), sock(context::instance().get(), ZMQ_SUB)
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
