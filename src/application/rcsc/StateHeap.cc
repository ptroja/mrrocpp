#include <iostream>

#include <boost/foreach.hpp>

#include "StateHeap.h"

namespace mrrocpp {
namespace mp {
namespace common {

void StateHeap::pushTargetName(const std::string & stateName)
{
	targetsHeap.push_back(stateName);
	// showing content
	//showHeapContent();
}

const std::string StateHeap::popTargetName()
{
	// showing content
	std::cerr << "poping..." << std::endl;
	showHeapContent();
	if(targetsHeap.empty())
		return "_STOP_";
	else
	{
		const std::string toReturn = targetsHeap.back();
		targetsHeap.pop_back();
		return toReturn;
	}
}

void StateHeap::showHeapContent()
{
	BOOST_FOREACH(const std::string & target, targetsHeap)
	{
		std::cout << "### on heap: #" << target << "#" << std::endl;
	}
}

} // namespace common
} // namespace mp
} // namespace mrrocpp
