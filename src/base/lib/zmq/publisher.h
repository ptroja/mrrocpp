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

	//! Publishing sockets.
	::zmq::socket_t tcp_sock, ipc_sock, inproc_sock;

	//! TCP port number.
	int port;

	//! Thread identificator for keep-alive.
	boost::thread tid;

	//! Keep-alive loop.
	void ping();

	//! Bind ephemeral TCP socket.
	int bind_ephemeral_tcp(::zmq::socket_t & sock);
};

} // namespace zmq
} // namespace lib
} // namespace mrrocpp

#endif /* ZMQ_PUBLISHER_H_ */
