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

namespace mrrocpp {
namespace lib {
namespace zmq {

class subscriber {
public:
	subscriber(const std::string & name);

private:
	const std::string remote_name;

	::zmq::socket_t sock;
};

} // namespace zmq
} // namespace lib
} // namespace mrrocpp

#endif /* ZMQ_SUBSCRIBER_H_ */
