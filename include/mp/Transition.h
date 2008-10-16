
#if !defined(_TRANSITION_H_)
#define _TRANSITION_H_

#include "mp/Condition.h"
#include "mp/StateHeap.h"

class Transition
{
	public:
		//Transition();
		Transition(char *cond, char *targetID, configurator &_config);
		Transition(const Transition &transition);
		~Transition();

		void showContent();
		bool getConditionResult();
		char * getTargetID(StateHeap &sh) const;

	private:
		char *targetID;
		Condition *condition;
};

#endif

