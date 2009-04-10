
#if !defined(_TRANSITION_H_)
#define _TRANSITION_H_

#include "mp/Condition.h"
#include "mp/StateHeap.h"

namespace mrrocpp {
namespace mp {
namespace common {


class Transition
{
	public:
		//Transition();
		Transition(char *cond, char *targetID, configurator &_config);
		Transition(const Transition &transition);
		~Transition();

		void showContent();
		bool getConditionResult();
		void setConditionResult(bool result);
		char * getTargetID(StateHeap &sh) const;
		char * getConditionDescription() const;

	private:
		char *targetID;
		Condition *condition;
};

} // namespace common
} // namespace mp
} // namespace mrrocpp


#endif

