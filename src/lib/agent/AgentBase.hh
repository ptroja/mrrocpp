#ifndef __AGENT_BASE_HH
#define __AGENT_BASE_HH

#include <string>

#include <boost/noncopyable.hpp>

#include "../messip/messip_dataport.h"

class AgentBase : boost::noncopyable {
private:
	//! Agent's name
	const std::string name;

protected:
	messip_channel_t * channel;

public:
	//! Get name of the agent
	const std::string & getName() const;

	AgentBase(const std::string & _name);
};

#endif /* __AGENT_BASE_HH */
