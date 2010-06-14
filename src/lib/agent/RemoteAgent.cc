/*
 * RemoteAgent.cc
 *
 *  Created on: May 25, 2010
 *      Author: ptroja
 */

#include <boost/thread/xtime.hpp>
#include <boost/thread.hpp>

#include "lib/exception.h"
#include <boost/throw_exception.hpp>
#include <boost/exception/errinfo_errno.hpp>
#include <boost/exception/errinfo_api_function.hpp>

#include <boost/serialization/string.hpp>

#include "RemoteAgent.h"

RemoteAgent::RemoteAgent(const std::string & _name, int retry, int sleep_ms) :
	AgentBase(_name)
{
	do {
#if defined(USE_MESSIP_SRR)
		channel = messip_channel_connect(NULL, getName().c_str(), MESSIP_NOTIMEOUT);
		if(channel == NULL)
#else /* USE_MESSIP_SRR */
		channel = name_open(getName().c_str(), NAME_FLAG_ATTACH_GLOBAL);
		if (channel == -1)
#endif /* USE_MESSIP_SRR */
		{
			if(--retry == 0) {
				std::cerr << "Unable to connect to '" << getName() << "'" << std::endl;
				BOOST_THROW_EXCEPTION(
					mrrocpp::lib::exception::System_error() <<
					boost::errinfo_errno(errno) <<
					boost::errinfo_api_function("messip_channel_connect")
				);
			} else {
				printf("."); fflush(stdout);

				// wait some time for process startup
				boost::this_thread::sleep(boost::posix_time::milliseconds(sleep_ms));
			}
		} else {
			// channel was successfully opened
			break;
		}
	} while(true);
}

RemoteAgent::~RemoteAgent() {
#if defined(USE_MESSIP_SRR)
	if(messip_channel_disconnect(channel, MESSIP_NOTIMEOUT) != 0) {
		BOOST_THROW_EXCEPTION(
			mrrocpp::lib::exception::System_error() <<
			boost::errinfo_errno(errno) <<
			boost::errinfo_api_function("messip_channel_disconnect")
		);
	}
#else /* USE_MESSIP_SRR */
	if(name_close(channel) == -1) {
		BOOST_THROW_EXCEPTION(
			mrrocpp::lib::exception::System_error() <<
			boost::errinfo_errno(errno) <<
			boost::errinfo_api_function("name_close")
		);
	}
#endif /* USE_MESSIP_SRR */
}

void RemoteAgent::Send(const xdr_oarchive<> & oa) {
	// do a non-blocking send
#if defined(USE_MESSIP_SRR)
	int ret = messip_send(channel, 0, 0,
			oa.get_buffer(), oa.getArchiveSize(),
			NULL, NULL, -1, MESSIP_NOTIMEOUT);
	// TODO:
	if (ret != 0) {
		BOOST_THROW_EXCEPTION(
			mrrocpp::lib::exception::System_error() <<
			boost::errinfo_errno(errno) <<
			boost::errinfo_api_function("messip_send")
		);
	}
#else /* USE_MESSIP_SRR */
	if(MsgSend(channel,
			oa.get_buffer(), oa.getArchiveSize(),
			NULL, 0)) {
		BOOST_THROW_EXCEPTION(
			mrrocpp::lib::exception::System_error() <<
			boost::errinfo_errno(errno) <<
			boost::errinfo_api_function("MsgSend")
		);
	}
#endif /* USE_MESSIP_SRR */
};
