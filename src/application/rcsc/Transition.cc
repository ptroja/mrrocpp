#include <string>

#include <cstdio>
#include <cstring>

#include "Transition.h"

namespace mrrocpp {
namespace mp {
namespace common {


Transition::Transition(const std::string & cond, const std::string & _targetID, lib::configurator &_config) :
	targetID(_targetID)
{
	this->condition = new Condition(cond, _config);
}

Transition::Transition(const Transition &transition) :
	targetID(transition.targetID)
{
	this->condition = new Condition(*(transition.condition));
}

Transition::~Transition()
{
	delete condition;
}

bool Transition::getConditionResult()
{
	return condition->checkCompareResult();;
}

void Transition::setConditionResult(bool result)
{
	condition->setResult(result);
}

const char * Transition::getTargetID(StateHeap &sh) const
{
	// TODO: reimplement ">>" operator as XML element
	const char *sp = ">>";
	if(strstr(targetID.c_str(), sp) != NULL)
	{
		// @bug possible memory corruption
		char * tmp = strdup(targetID.c_str());
		char *nextState = strtok(tmp, sp);
		sh.pushTargetName(strtok(NULL, sp));
		free(tmp);
		return nextState;
	}
	else
		return targetID.c_str();
}

const std::string & Transition::getConditionDescription() const
{
	return condition->getCondDesc();
}

void Transition::showContent() const
{
	std::cerr << ">> Condition: #" << condition->getCondDesc() << "#" << std::endl;
	std::cerr << "Target state: #" << targetID << "#" << std::endl;
	std::cerr << ">> Condition result: " << condition->checkCompareResult() << std::endl;
}

} // namespace common
} // namespace mp
} // namespace mrrocpp


