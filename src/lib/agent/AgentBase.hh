#ifndef __AGENT_BASE_HH
#define __AGENT_BASE_HH

#include <string>

#include <boost/noncopyable.hpp>

#if defined(USE_MESSIP_SRR)
#include "lib/messip/messip.h"
#else /* USE_MESSIP_SRR */
#include <sys/iofunc.h>
#include <sys/dispatch.h>
#endif /* USE_MESSIP_SRR */

class AgentBase : boost::noncopyable {
private:
	//! Agent's name
	const std::string name;

protected:
#if defined(USE_MESSIP_SRR)
	//! channel id for both server and client
	messip_channel_t * channel;
#endif /* USE_MESSIP_SRR */

public:
	//! Get name of the agent
	const std::string & getName() const;

	AgentBase(const std::string & _name);
};

#endif /* __AGENT_BASE_HH */
