/*
 * registry.cc
 *
 *  Created on: Feb 13, 2012
 *      Author: ptroja
 */

#include <string>

#include <boost/thread/once.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/lexical_cast.hpp>

#include <boost/asio/ip/host_name.hpp>

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
	: sock(context::instance().get(), ZMQ_REQ),
	  keep_alive_sock(context::instance().get(), ZMQ_PUB)
{
	std::string synch_address = "tcp://localhost:";
	synch_address += boost::lexical_cast<std::string>(mrrocpp::lib::zmq::registry_port);

	sock.connect(synch_address.c_str());

	std::string keep_alive_address = "tcp://localhost:";
	keep_alive_address += boost::lexical_cast<std::string>(mrrocpp::lib::zmq::keep_alive_port);

	keep_alive_sock.connect(keep_alive_address.c_str());
}

void registry::register_name(const std::string & name, int port)
{
	location me;

	me.type = location::REGISTER;
	me.host = boost::asio::ip::host_name();
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

		send(keep_alive_sock, ping_message);
	}
}

} // namespace zmq
} // namespace lib
} // namespace mrrocpp
