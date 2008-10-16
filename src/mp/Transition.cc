
#include <stdio.h>
#include <string.h>

#include "mp/Transition.h"

Transition::Transition(char *cond, char *targetID, configurator &_config)
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

char * Transition::getTargetID(StateHeap &sh) const
{
	char *sp = ">>";
	if(strstr(targetID, sp) != NULL)
	{
		char *nextState = strtok(targetID, sp);
		sh.pushTargetName(strtok(NULL, sp));
		return nextState;
	}
	else
		return targetID;
}
		
void Transition::showContent()
{
	printf(">> Condition: #%s#\n>> Target state: #%s#\n", condition->getCondDesc(), targetID);
	printf(">> Codition result: %d\n", condition->checkCompareResult());
//	std::cout<<">> Condition: "<<condition<<std::endl<<
//		">> Target State: "<<targetID<<std::endl;
}

