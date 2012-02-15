/*
 * context.h
 *
 *  Created on: Feb 13, 2012
 *      Author: ptroja
 */

#ifndef ZMQ_CONTEXT_H_
#define ZMQ_CONTEXT_H_

#include <boost/thread/once.hpp>

#include <zmq.hpp>

namespace mrrocpp {
namespace lib {
namespace zmq {

class context {
public:
	::zmq::context_t & get();

private:
	context();

	::zmq::context_t cnx;

public:
	static context & instance();

private:
	static void create();

	static context * global;

	static boost::once_flag once;
};

} // namespace zmq
} // namespace lib
} // namespace mrrocpp

#endif /* ZMQ_CONTEXT_H_ */
