
#include <stdio.h>
#include <string.h>

#include "mp/StateHeap.h"


StateHeap::StateHeap()
{
	targetsHeap = NULL;
}

StateHeap::~StateHeap()
{
	if(targetsHeap != NULL)
		delete targetsHeap;
}

void StateHeap::pushTargetName(char * stateName)
{
	char *toAdd = new char[strlen(stateName)];
	strcpy(toAdd, stateName);
	if(targetsHeap == NULL)
		targetsHeap = new std::list<char *>();
	targetsHeap->push_back(toAdd);
	// showing content
	//showHeapContent();
}

char * StateHeap::popTargetName()
{
	// showing content
	printf("poping..\n");
	showHeapContent();
	if(targetsHeap == NULL || 
			targetsHeap->empty())
		return "STOP";
	else
	{
		char *toReturn = new char[strlen(targetsHeap->back())];
		strcpy(toReturn, targetsHeap->back());
		targetsHeap->pop_back();
		return toReturn;
	}
}

void StateHeap::showHeapContent()
{
	for(std::list<char *>::iterator it = targetsHeap->begin(); it != targetsHeap->end(); ++it)
	{
		printf("### on heap: #%s#\n", (*it));
	}
}

