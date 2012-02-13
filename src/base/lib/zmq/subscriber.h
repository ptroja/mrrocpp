/*
 * subscriber.h
 *
 *  Created on: Feb 13, 2012
 *      Author: ptroja
 */

#ifndef ZMQ_SUBSCRIBER_H_
#define ZMQ_SUBSCRIBER_H_

#include <string>

#include <zmq.hpp>

#include "../xdr/xdr_iarchive.hpp"

namespace mrrocpp {
namespace lib {
namespace zmq {

class subscriber {
public:
	//! Constructor.
	subscriber(const std::string & name);

private:
	//! Name of the publication topic.
	const std::string topic_name;

	//! Socket for data receiving.
	::zmq::socket_t sock;

	//! Cross-platform serialization archive.
	xdr_iarchive<> xdr_ia;

public:
	//! Receive data.
	template<typename T>
	void receive(T & data)
	{
		char blob[16*1024];

		//! Pre-allocated ZMQ message.
		::zmq::message_t msg(blob, sizeof(blob), NULL);

		sock.recv(&msg);

		xdr_ia.set_buffer((const char *) msg.data(), msg.size());

		xdr_ia >> data;

		std::cout << "receive('" << data << "', " << msg.size() << ")" << std::endl;
	}
};

} // namespace zmq
} // namespace lib
} // namespace mrrocpp

#endif /* ZMQ_SUBSCRIBER_H_ */
