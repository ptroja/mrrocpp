
#if !defined(_STATE_HEAP_H_)
#define _STATE_HEAP_H_

#include <list>

class StateHeap
{
	public:
		StateHeap();
		~StateHeap();

		void pushTargetName(char * stateName);
		char * popTargetName();

		void showHeapContent();

	private:
		std::list<char *> *targetsHeap;
};

#endif

