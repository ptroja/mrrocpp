#ifndef __REMOTE_AGENT_HH
#define __REMOTE_AGENT_HH

#include <boost/serialization/string.hpp>

#include "AgentBase.hh"
#include "../xdr_oarchive.hpp"

#if defined(USE_MESSIP_SRR)
#include "../messip/messip.h"
#else /* USE_MESSIP_SRR */
#include <sys/iofunc.h>
#include <sys/dispatch.h>
#include <sys/neutrino.h>
#endif /* USE_MESSIP_SRR */

class RemoteAgent : public AgentBase {
private:
#if !defined(USE_MESSIP_SRR)
	//! remove server channel id
	int channel;
#endif
public:
	/**
	 * Set the data of given buffer
	 * @param name buffer name
	 * @param the data
	 */
	template <class T>
	void Set(const std::string & name, const T & item) {
		xdr_oarchive<4096> oa;
		oa << name;
		oa << item;

		// do a non-blocking send
#if defined(USE_MESSIP_SRR)
		int ret = messip_send(channel, 0, 0,
				oa.get_buffer(), oa.getArchiveSize(),
				NULL, NULL, -1, MESSIP_NOTIMEOUT);
		// TODO:
		if (ret != 0) throw;
#else /* USE_MESSIP_SRR */
		if(MsgSend(channel,
				oa.get_buffer(), oa.getArchiveSize(),
				NULL, 0)) {
			perror("MsgSend()");
			// TODO: throw
			throw;
		}
#endif /* USE_MESSIP_SRR */
	};

	RemoteAgent(const std::string & _name) :
		AgentBase(_name)
	{
#if defined(USE_MESSIP_SRR)
		channel = messip_channel_connect(NULL, getName().c_str(), MESSIP_NOTIMEOUT);
		if(channel == NULL) {
			// TODO:
			throw;
		}
#else /* USE_MESSIP_SRR */
		channel = name_open(getName().c_str(), 0 /*NAME_FLAG_ATTACH_GLOBAL*/);
		if (channel == -1) {
			perror("name_open()");
			// TODO:
			throw;
		}
#endif /* USE_MESSIP_SRR */
	}

	virtual ~RemoteAgent() {
#if defined(USE_MESSIP_SRR)
		if(messip_channel_disconnect(channel, MESSIP_NOTIMEOUT) != 0) {
			// TODO:
			throw;
		}
#else /* USE_MESSIP_SRR */
		if(name_close(channel) == -1) {
			perror("name_close()");
			// TODO:
			throw;
		}
#endif /* USE_MESSIP_SRR */
	}
};

#endif /* __REMOTE_AGENT_HH */
