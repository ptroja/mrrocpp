#if !defined(_TRANSITION_H_)
#define _TRANSITION_H_

#include "Condition.h"
#include "StateHeap.h"

namespace mrrocpp {
namespace mp {
namespace common {


class Transition
{
	public:
		//Transition();
		Transition(const std::string & cond, const std::string & targetID, lib::configurator &_config);
		Transition(const Transition &transition);
		~Transition();

		void showContent() const;
		bool getConditionResult();
		void setConditionResult(bool result);
		const char * getTargetID(StateHeap &sh) const;
		const std::string & getConditionDescription() const;

	private:
		std::string targetID;
		Condition * condition;
};

} // namespace common
} // namespace mp
} // namespace mrrocpp


#endif

