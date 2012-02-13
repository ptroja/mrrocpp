/*
 * registry.cc
 *
 *  Created on: Feb 13, 2012
 *      Author: ptroja
 */

#include <string>

#include <boost/thread/once.hpp>
#include <boost/thread/mutex.hpp>

#include <unistd.h>

#include "registry.h"
#include "context.h"

namespace mrrocpp {
namespace lib {
namespace zmq {

registry & registry::instance()
{
	boost::call_once(&create, once);

	return *global;
}

boost::once_flag registry::once = BOOST_ONCE_INIT;

registry * registry::global = NULL;

void registry::create()
{
	global = new registry();
}

registry::registry()
	: sock(context::instance().get(), ZMQ_REQ)
{
	sock.connect("tcp://localhost:5555");
}

void registry::register_name(const std::string & name, int port)
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

void registry::unregister_name(const std::string & name)
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

location registry::locate_name(const std::string & name)
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

void registry::ping(const std::string & name)
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

} // namespace zmq
} // namespace lib
} // namespace mrrocpp
