#ifndef __REMOTE_AGENT_H
#define __REMOTE_AGENT_H

#include <string>
#include <unistd.h>

#include <boost/serialization/string.hpp>

#include "AgentBase.h"
#include "base/lib/xdr/xdr_oarchive.hpp"
#include "base/lib/impconst.h"

#include "../messip/messip.h"
// TODO: rewrite with messip dataport wrapper
#include "../messip/messip_dataport.h"

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

		if (ret != 0) {
			throw std::logic_error("Could not send to remote agent");
		}
	};

	RemoteAgent(const std::string & _name) :
		AgentBase(_name)
	{
		// nawiazanie komunikacji z ECP
		unsigned int tmp = 0;
		// kilka sekund  (~1) na otworzenie urzadzenia

		while ((channel = messip::port_connect(_name)) == NULL) {
			if ((tmp++) < lib::CONNECT_RETRY) {
				usleep(lib::CONNECT_DELAY);
			} else {
				fprintf(stderr, "Connect to failed at channel '%s'\n", _name.c_str());
				throw std::logic_error("Connect from remote agent failed");
			}
		}
	}

	virtual ~RemoteAgent()
	{
		if(messip::port_disconnect(channel, MESSIP_NOTIMEOUT) != 0) {
			// TODO: check for results
			throw std::logic_error("Disconnect from remote agent failed");
		}
	}
};

template <class T>
class OutputBuffer {
private:
	//! name of the buffer
	const std::string name;

	//! owner of the buffer
	RemoteAgent &owner;

public:
	//! Construct remote buffer proxy
	OutputBuffer(RemoteAgent & _owner, const std::string & _name)
		: name(_name), owner(_owner)
	{
	}

	//! Set the contents of the remove buffer
	void Send(const T & data) {
		xdr_oarchive<> oa;
		oa << name;
		oa << data;

		owner.Send(oa);
	}
};

#endif /* __REMOTE_AGENT_H */
