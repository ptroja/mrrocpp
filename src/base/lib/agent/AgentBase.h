#ifndef __AGENT_BASE_H
#define __AGENT_BASE_H

#include <string>

#include <boost/noncopyable.hpp>

class AgentBase : boost::noncopyable {
private:
	//! Agent's name
	const std::string name;

public:
	//! Get name of the agent
	const std::string & getName() const;

	AgentBase(const std::string & _name);
};

#endif /* __AGENT_BASE_H */
