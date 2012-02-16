/*
 * registry.h
 *
 *  Created on: Feb 13, 2012
 *      Author: ptroja
 */

#ifndef ZMQ_REGISTRY_H_
#define ZMQ_REGISTRY_H_

#include <string>

#include <boost/thread/mutex.hpp>
#include <boost/thread/once.hpp>

#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>

#include <zmq.hpp>

#include "location.h"

namespace mrrocpp {
namespace lib {
namespace zmq {

//! Port number for synchronous requests.
const unsigned int registry_port = 5555;

//! Port number for asynchronous keep-alive messages.
const unsigned int keep_alive_port = 5556;

template<typename T>
void send(::zmq::socket_t & sock, const T & data)
{
	std::ostringstream os;

	boost::archive::text_oarchive oa(os);

	oa << data;

	::zmq::message_t msg(os.str().size());

	memcpy(msg.data(), os.str().c_str(), os.str().size());

	sock.send(msg);
}

template<typename T>
void recv(::zmq::socket_t & sock, T & data)
{
	::zmq::message_t msg;

	sock.recv(&msg);

	std::istringstream is((char *) msg.data());

	boost::archive::text_iarchive ia(is);

	ia >> data;
}

class registry : private boost::noncopyable {
public:
	static registry & instance();

	void register_name(const std::string & name, int port);

	void unregister_name(const std::string & name);

	location locate_name(const std::string & name);

	void ping(const std::string & name);

private:
	registry();

	::zmq::socket_t sock, keep_alive_sock;

	//! Access protection.
	boost::mutex mtx;

private:
	static void create();

	static registry * global;

	static boost::once_flag once;
};

} // namespace zmq
} // namespace lib
} // namespace mrrocpp

#endif /* ZMQ_REGISTRY_H_ */
