#ifndef __REMOTE_AGENT_H
#define __REMOTE_AGENT_H

#include <string>

#include "AgentBase.h"

#include "../messip/messip_dataport.h"

namespace mrrocpp {
namespace lib {
namespace agent {

/**
 * Remote agent proxy
 */
class RemoteAgent : public AgentBase {
public:
	//! Connect to remote agent
	RemoteAgent(const std::string & _name);

	//! Check connection with ping message
	void Ping();

	//! Disconnect from remote agent
	virtual ~RemoteAgent();

private:
	//! remote server channel id
	messip_channel_t * channel;

	/**
	 * Set the data of given buffer
	 * @param name buffer name
	 * @param the data
	 * @todo this should not be public method but friending with template OutputBuffer is tricky
	 */
	void Send(const xdr_oarchive<> & oa);

	template <class T> friend class OutputBuffer;
};

} // namespace agent
} // namespace lib
} // namespace mrrocpp

#endif /* __REMOTE_AGENT_H */
