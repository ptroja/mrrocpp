/*
 * zmqpp.h
 *
 *  Created on: Feb 7, 2012
 *      Author: ptroja
 */

#ifndef ZMQPP_H_
#define ZMQPP_H_

#include <string>
#include <sstream>

#include <boost/noncopyable.hpp>
#include <boost/thread/once.hpp>

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/string.hpp>

#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>

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

class location {
public:
	enum { REGISTER, UNREGISTER, QUERY, ACK, NACK } type;
	std::string name;
	std::string host;
	int port;
	int pid;
private:
	//! Give access to boost::serialization framework
	friend class boost::serialization::access;

	//! Serialization of the data structure
	template <class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		ar & type;
		switch(type) {
			case REGISTER:
			case ACK:
			case NACK:
				ar & name;
				ar & host;
				ar & port;
				ar & pid;
				break;
			case QUERY:
			case UNREGISTER:
				ar & name;
				break;
		}
	}

	friend std::ostream& operator<< (std::ostream& stream, const location & me)
	{
		switch(me.type) {
			case REGISTER:
				stream << "REGISTER:" << me.name << "@" << me.host << ":" << me.port << "/" << me.pid;
				break;
			case ACK:
				stream << "ACK:" << me.name;
				break;
			case NACK:
				stream << "NACK:" << me.name;
				break;
			case QUERY:
				stream << "QUERY:" << me.name;
				break;
			case UNREGISTER:
				stream << "UNREGISTER:" << me.name;
				break;
			default:
				stream << "?:" << me.name;
				break;
		}

		return stream;
	}
};

class locator : private boost::noncopyable {
public:
	static locator & instance();

	void register_name(const std::string & name, int port);

	void unregister_name(const std::string & name);

	location locate_name(const std::string & name);

private:
	locator();

	zmq::socket_t sock;

private:
	static void create();

	static locator * global;

	static boost::once_flag once;
};

int bind_ephemeral_tcp(zmq::socket_t & sock);

class receiver {
public:
	receiver(const std::string & name);

	~receiver();

private:
	const std::string my_name;

	zmq::socket_t tcp_sock, ipc_sock, inproc_sock;

	int port;
};

class sender {
public:
	sender(const std::string & name);

private:
	const std::string remote_name;

	zmq::socket_t sock;
};

template<typename T>
void send(zmq::socket_t & sock, const T & data)
{
	std::ostringstream os;

	boost::archive::text_oarchive oa(os);

	oa << data;

	zmq::message_t msg(os.str().size());

	memcpy(msg.data(), os.str().c_str(), os.str().size());

	sock.send(msg);
}

template<typename T>
void recv(zmq::socket_t & sock, T & data)
{
	zmq::message_t msg;

	sock.recv(&msg);

	std::istringstream is((char *) msg.data());

	boost::archive::text_iarchive ia(is);

	ia >> data;
}

}

#endif /* ZMQPP_H_ */
