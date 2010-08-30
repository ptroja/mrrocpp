#include "AgentBase.h"

AgentBase::AgentBase(const std::string & _name) :
	name(_name)
{
}

const std::string & AgentBase::getName() const
{
	return name;
}
