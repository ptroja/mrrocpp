#ifndef __REMOTE_AGENT_H
#define __REMOTE_AGENT_H

#include <string>

#include "AgentBase.h"

#include "lib/xdr/xdr_oarchive.hpp"

#if defined(USE_MESSIP_SRR)
#include "lib/messip/messip.h"
#else /* USE_MESSIP_SRR */
#include <sys/iofunc.h>
#include <sys/dispatch.h>
#include <sys/neutrino.h>
#endif /* USE_MESSIP_SRR */

class RemoteAgent : public AgentBase {
private:
#if defined(USE_MESSIP_SRR)
	//! remote server channel id
	messip_channel_t * channel;
#else
	//! remote server channel id
	int channel;
#endif
public:
	/**
	 * Set the data of given buffer
	 * @param name buffer name
	 * @param the data
	 * @todo this should not be public method but friending with template RemoteBuffer is tricky
	 */
	void Send(const xdr_oarchive<> & oa);

	/**
	 * Constructor
	 * @param _name name of the remote agent
	 * @param retry number of retries to connect
	 * @param sleep_ms interval between trying to connect in milliseconds
	 */
	RemoteAgent(const std::string & _name, int retry = 50, int sleep_ms = 100);

	/**
	 * Destructor
	 */
	virtual ~RemoteAgent();
};

#endif /* __REMOTE_AGENT_H */
