#include "AgentBase.h"

namespace mrrocpp {
namespace lib {
namespace agent {

AgentBase::AgentBase(const std::string & _name) :
	name(_name)
{
}

const std::string & AgentBase::getName() const
{
	return name;
}

} // namespace agent
} // namespace lib
} // namespace mrrocpp
