
#include <cstdio>
#include <cstring>

#include "StateHeap.h"

namespace mrrocpp {
namespace mp {
namespace common {



StateHeap::StateHeap()
{
	targetsHeap = NULL;
}

StateHeap::~StateHeap()
{
	if(targetsHeap != NULL)
		delete targetsHeap;
}

void StateHeap::pushTargetName(const char * stateName)
{
	char *toAdd = new char[strlen(stateName)];
	strcpy(toAdd, stateName);
	if(targetsHeap == NULL)
		targetsHeap = new std::list<const char *>();
	targetsHeap->push_back(toAdd);
	// showing content
	//showHeapContent();
}

const char * StateHeap::popTargetName()
{
	// showing content
	printf("poping..\n");
	showHeapContent();
	if(targetsHeap == NULL ||
			targetsHeap->empty())
		return "_STOP_";
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
	for(std::list<const char *>::iterator it = targetsHeap->begin(); it != targetsHeap->end(); ++it)
	{
		printf("### on heap: #%s#\n", (*it));
	}
}
} // namespace common
} // namespace mp
} // namespace mrrocpp



