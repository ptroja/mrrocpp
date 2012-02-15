/*
 * publisher.h
 *
 *  Created on: Feb 13, 2012
 *      Author: ptroja
 */

#ifndef ZMQ_PUBLISHER_H_
#define ZMQ_PUBLISHER_H_

#include <string>
#include <boost/thread/thread.hpp>
#include <zmq.hpp>

#include "../xdr/xdr_oarchive.hpp"

namespace mrrocpp {
namespace lib {
namespace zmq {

class publisher {
public:
	//! Constructor.
	publisher(const std::string & name);

	//! Destructor.
	~publisher();

private:
	//! Name.
	const std::string my_name;

	//! Publishing socket.
	::zmq::socket_t sock;

	//! TCP port number.
	int port;

	//! Thread identificator for keep-alive.
	boost::thread tid;

	//! Keep-alive loop.
	void ping();

	//! Bind ephemeral TCP socket.
	int bind_ephemeral_tcp(::zmq::socket_t & sock);

public:
	//! Send data.
	template<typename T>
	void send(const T & data)
	{
		// Cross-platform serialization archive.
		xdr_oarchive<> xdr_oa;

		// Serialize data.
		xdr_oa << data;

		// Build ZMQ message.
		::zmq::message_t msg((void *) xdr_oa.get_buffer(), xdr_oa.getArchiveSize(), NULL);

		// Copy data to ZMQ message.
		memcpy(msg.data(), xdr_oa.get_buffer(), xdr_oa.getArchiveSize());

		std::cout << "send('" << data << "', " << xdr_oa.getArchiveSize() << ")" << std::endl;

		sock.send(msg, ZMQ_NOBLOCK);
	}
};

} // namespace zmq
} // namespace lib
} // namespace mrrocpp

#endif /* ZMQ_PUBLISHER_H_ */
