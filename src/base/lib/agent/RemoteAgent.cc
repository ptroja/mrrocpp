/*
 * RemoteAgent.cc
 *
 *  Created on: Dec 10, 2011
 *      Author: ptroja
 */

#include <exception>
#include <string>
#include <cstdio>
#include <unistd.h>

#include <boost/thread/thread.hpp>

#include "RemoteAgent.h"

#include "../messip/messip.h"
// TODO: rewrite with messip dataport wrapper
#include "../messip/messip_dataport.h"

#include <boost/serialization/string.hpp>
#include "base/lib/xdr/xdr_oarchive.hpp"

#include "base/lib/impconst.h"

namespace mrrocpp {
namespace lib {
namespace agent {

void RemoteAgent::Send(const xdr_oarchive <> & oa)
{
	// do a non-blocking send
	int ret = messip_send(channel, 0, 0, oa.get_buffer(), oa.getArchiveSize(), NULL, NULL, -1, MESSIP_NOTIMEOUT);

	if (ret != 0) {
		throw std::logic_error("Could not send to remote agent");
	}
}

void RemoteAgent::Ping()
{
	if(messip::port_ping(channel) != 0)
		throw std::runtime_error("Pinging remote agent failed");
}

RemoteAgent::RemoteAgent(const std::string & _name) :
		AgentBase(_name)
{
	// nawiazanie komunikacji z ECP
	unsigned int tmp = 0;
	// kilka sekund  (~1) na otworzenie urzadzenia

	while ((channel = messip::port_connect(_name)) == NULL) {
		if ((tmp++) < lib::CONNECT_RETRY) {
			boost::this_thread::sleep(lib::CONNECT_DELAY);
		} else {
			fprintf(stderr, "Connect to failed at channel '%s'\n", _name.c_str());
			throw std::runtime_error("Connect to remote agent failed");
		}
	}

//	// Verify if channel is ready to transmit data
//	if (messip::port_ping(channel)) {
//		throw std::logic_error("Ping to remote agent failed");
//	}
}

RemoteAgent::~RemoteAgent()
{
	if (messip::port_disconnect(channel, MESSIP_NOTIMEOUT) != 0) {
		// TODO: check for results
	}
}

} // namespace agent
} // namespace lib
} // namespace mrrocpp
