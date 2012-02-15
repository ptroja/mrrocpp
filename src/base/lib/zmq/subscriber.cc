/*
 * subscriber.cc
 *
 *  Created on: Feb 13, 2012
 *      Author: ptroja
 */

#include <stdexcept>

#include <unistd.h>

#include "subscriber.h"
#include "context.h"
#include "location.h"
#include "registry.h"

namespace mrrocpp {
namespace lib {
namespace zmq {

subscriber::subscriber(const std::string & remote_name_)
	: topic_name(remote_name_),
	  sock(context::instance().get(), ZMQ_SUB)
{
	// Get own location.
	char hostname[256];

	if(gethostname(hostname, 256) == -1) {
		throw std::runtime_error("Could not get hostname");
	}

	location remote = registry::instance().locate_name(topic_name);

	// Build address with string stream.
	std::ostringstream os;

	// Check if we are on the same machine.
	if(remote.host == hostname) {
		// Check if we are within the same process.
		if(remote.pid == (int) getpid()) {
			// Connect with INPROC transport.
			os << "inproc://" << topic_name;
		} else {
			// Connect with IPC transport.
			os << "ipc:///tmp/.zmq_" << remote.pid << "_" << topic_name;
		}
	} else {
		// Connect with TCP transport.
		os << "tcp://" << remote.host << ":" << remote.port;
	}

	// Connect.
	sock.connect(os.str().c_str());

	// Recive all messages by default.
	sock.setsockopt(ZMQ_SUBSCRIBE, NULL, 0);
}

} // namespace zmq
} // namespace lib
} // namespace mrrocpp
