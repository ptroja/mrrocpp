
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
		bool setConditionResult(bool result);
		char * getTargetID(StateHeap &sh) const;
		char * getConditionDescription() const;

	private:
		char *targetID;
		Condition *condition;
};

#endif

