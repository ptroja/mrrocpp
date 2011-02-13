#ifndef __REMOTE_AGENT_H
#define __REMOTE_AGENT_H

#include <string>

#include <boost/serialization/string.hpp>

#include "AgentBase.h"
#include "base/lib/xdr/xdr_oarchive.hpp"

#include "../messip/messip.h"

class RemoteAgent : public AgentBase {
private:
	//! remote server channel id
	messip_channel_t * channel;

public:
	/**
	 * Set the data of given buffer
	 * @param name buffer name
	 * @param the data
	 * @todo this should not be public method but friending with template RemoteBuffer is tricky
	 */
	void Send(const xdr_oarchive<> & oa) {
		// do a non-blocking send
		int ret = messip_send(channel, 0, 0,
				oa.get_buffer(), oa.getArchiveSize(),
				NULL, NULL, -1, MESSIP_NOTIMEOUT);

		// TODO: check for results
		if (ret != 0) throw;
	};

	RemoteAgent(const std::string & _name) :
		AgentBase(_name)
	{
		channel = messip_channel_connect(NULL, getName().c_str(), MESSIP_NOTIMEOUT);
		if(channel == NULL) {
			// TODO: check for results
			throw;
		}
	}

	virtual ~RemoteAgent()
	{
		if(messip_channel_disconnect(channel, MESSIP_NOTIMEOUT) != 0) {
			// TODO: check for results
			throw;
		}
	}
};

template <class T>
class RemoteBuffer {
private:
	//! name of the buffer
	const std::string name;

	//! owner of the buffer
	RemoteAgent &owner;

public:
	//! Construct remote buffer proxy
	RemoteBuffer(RemoteAgent & _owner, const std::string & _name)
		: name(_name), owner(_owner)
	{
	}

	//! Set the contents of the remove buffer
	void Set(const T & data) {
		xdr_oarchive<> oa;
		oa << name;
		oa << data;

		owner.Send(oa);
	}
};

#endif /* __REMOTE_AGENT_H */
