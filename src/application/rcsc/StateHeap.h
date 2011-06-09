#if !defined(_STATE_HEAP_H_)
#define _STATE_HEAP_H_

#include <list>

namespace mrrocpp {
namespace mp {
namespace common {

class StateHeap
{
	public:
		void pushTargetName(const std::string & stateName);
		const std::string popTargetName();

		void showHeapContent();

	private:
		std::list<std::string> targetsHeap;
};

} // namespace common
} // namespace mp
} // namespace mrrocpp

#endif
