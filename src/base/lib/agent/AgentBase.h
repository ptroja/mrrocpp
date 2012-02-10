#ifndef __AGENT_BASE_H
#define __AGENT_BASE_H

#include <string>

#include <boost/noncopyable.hpp>

namespace mrrocpp {
namespace lib {
namespace agent {

/**
 * Base class for agent and its proxy
 */
class AgentBase : boost::noncopyable {
private:
	//! Agent's name
	const std::string name;

public:
	//! Get name of the agent
	const std::string & getName() const;

	//! Constructor
	//! @param _name name of the agent
	AgentBase(const std::string & _name);
};

} // namespace agent
} // namespace lib
} // namespace mrrocpp

#endif /* __AGENT_BASE_H */
