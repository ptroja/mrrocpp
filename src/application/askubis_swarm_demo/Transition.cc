
#include <cstdio>
#include <cstring>

#include "Transition.h"

namespace mrrocpp {
namespace mp {
namespace common {


Transition::Transition(const char *cond, const char *targetID, lib::configurator &_config)
{
	int size = strlen(targetID) + 1;
	this->targetID = new char[size];
	strcpy(this->targetID, targetID);
	this->condition = new Condition(cond, _config);
}

Transition::Transition(const Transition &transition)
{
	int size = strlen(transition.targetID) + 1;
	this->targetID = new char[size];
	strcpy(this->targetID, transition.targetID);
	this->condition = new Condition(*(transition.condition));
}

Transition::~Transition()
{
	delete condition;
	delete[] targetID;
}

bool Transition::getConditionResult()
{
	bool result = condition->checkCompareResult();
	return result;
}

void Transition::setConditionResult(bool result)
{
	condition->setResult(result);
}

const char * Transition::getTargetID(StateHeap &sh) const
{
	// TODO: reimplement ">>" operator as XML element
	const char *sp = ">>";
	if(strstr(targetID, sp) != NULL)
	{
		char *nextState = strtok(targetID, sp);
		sh.pushTargetName(strtok(NULL, sp));
		return nextState;
	}
	else
		return targetID;
}

std::string Transition::getConditionDescription() const
{
	return std::string(condition->getCondDesc());
}

void Transition::showContent()
{
	printf(">> Condition: #%s#\n>> Target state: #%s#\n", condition->getCondDesc(), targetID);
	printf(">> Codition result: %d\n", condition->checkCompareResult());
//	std::cout<<">> Condition: "<<condition<<std::endl<<
//		">> Target State: "<<targetID<<std::endl;
}

} // namespace common
} // namespace mp
} // namespace mrrocpp


