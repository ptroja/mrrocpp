
#if !defined(_STATE_HEAP_H_)
#define _STATE_HEAP_H_

#include <list>

namespace mrrocpp {
namespace mp {
namespace common {


class StateHeap
{
	public:
		StateHeap();
		~StateHeap();

		void pushTargetName(const char * stateName);
		const char * popTargetName();

		void showHeapContent();

	private:
		std::list<const char *> *targetsHeap;
};

} // namespace common
} // namespace mp
} // namespace mrrocpp


#endif

