#ifndef __REMOTE_AGENT_HH
#define __REMOTE_AGENT_HH

#include <boost/serialization/string.hpp>

#include "AgentBase.hh"
#include "../messip/messip_dataport.h"
#include "../xdr_oarchive.hpp"

class RemoteAgent : public AgentBase {
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

		int ret = messip_send(channel, 0, 0,
				oa.get_buffer(), oa.getArchiveSize(),
				NULL, NULL, -1, MESSIP_NOTIMEOUT);
		if (ret != 0) throw;
	};

	RemoteAgent(const std::string & _name) :
		AgentBase(_name)
	{
		channel = messip::port_connect(getName());
	}

	virtual ~RemoteAgent() {
		messip::port_disconnect(channel);
	}
};

#endif /* __REMOTE_AGENT_HH */
