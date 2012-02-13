/*
 * context.cc
 *
 *  Created on: Feb 13, 2012
 *      Author: ptroja
 */

#include "context.h"

#include <zmq.hpp>

namespace mrrocpp {
namespace lib {
namespace zmq {

boost::once_flag context::once = BOOST_ONCE_INIT;

context * context::global = NULL;

context & context::instance()
{
	boost::call_once(&create, once);

	return *global;
}

void context::create()
{
	global = new context();
}

context::context()
	: cnx(1)
{
}

::zmq::context_t & context::get()
{
	return cnx;
}

} // namespace zmq
} // namespace lib
} // namespace mrrocpp
