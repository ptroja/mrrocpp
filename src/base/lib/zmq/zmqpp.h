/*
 * zmqpp.h
 *
 *  Created on: Feb 7, 2012
 *      Author: ptroja
 */

#ifndef ZMQPP_H_
#define ZMQPP_H_

#include <string>

#include <boost/noncopyable.hpp>
#include <boost/thread/once.hpp>

#include <zmq.hpp>

namespace zmqpp {

class context : private boost::noncopyable {
public:
	zmq::context_t & get();

private:
	context();

	zmq::context_t cnx;

public:
	static context & instance();

private:
	static void create();

	static context * global;

	static boost::once_flag once;
};

class locator : private boost::noncopyable {
public:
	static locator & instance();

	void register_name(const std::string & name, int port);

	void unregister_name(const std::string & name);

	std::string locate_name(const std::string & name);

private:
	locator();

	zmq::socket_t sock;

private:
	static void create();

	static locator * global;

	static boost::once_flag once;
};

int bind_ephemeral(zmq::socket_t & sock);

class receiver {
public:
	receiver(const std::string & name);

private:
	const std::string my_name;

	zmq::socket_t sock;

	int port;
};

class sender {
public:
	sender(const std::string & name);

private:
	const std::string remote_name;

	zmq::socket_t sock;
};


}

#endif /* ZMQPP_H_ */
