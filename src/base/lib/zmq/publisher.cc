/*
 * publisher.cc
 *
 *  Created on: Feb 13, 2012
 *      Author: ptroja
 */

#include <cstdio>

#include <boost/bind.hpp>
#include <boost/thread/thread_time.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp>

#include "publisher.h"
#include "registry.h"
#include "context.h"

namespace mrrocpp {
namespace lib {
namespace zmq {

publisher::publisher(const std::string & name)
	: my_name(name),
	  sock(context::instance().get(), ZMQ_PUB)
{
	// Bind to TCP socket.
	port = bind_ephemeral_tcp(sock);

	// Build address with string stream.
	std::ostringstream ipc_address, inproc_address;

	// INPROC transport address.
	ipc_address << "ipc:///tmp/.zmq_" << (int) getpid() << "_" << name;

	sock.bind(ipc_address.str().c_str());

	inproc_address << "inproc://" << name;

	sock.bind(inproc_address.str().c_str());

	// Register name.
	registry::instance().register_name(my_name, port);

	// High-water mark setting.
	const uint64_t hwm_value = 1;

	// Set high-water mark action.
	sock.setsockopt(ZMQ_HWM, &hwm_value, sizeof(hwm_value));

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
	registry::instance().unregister_name(my_name);
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
		registry::instance().ping(my_name);
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

int publisher::bind_ephemeral_tcp(::zmq::socket_t & sock)
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

} // namespace zmq
} // namespace lib
} // namespace mrrocpp
