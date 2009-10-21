// ----------------------------------------------------------------------
// Definicje klas wykorzystywanych przy implementacji
// automatu skonczonego
// Autor: Marek Kisiel
// ----------------------------------------------------------------------

#if !defined(_STATE_H_)
#define _STATE_H_

#include <list>

#include "ecp_mp/ecp_mp_t_fsautomat.h"
#include "lib/impconst.h"
#include "mp/Transition.h"


namespace mrrocpp {
namespace mp {
namespace common {



//enum StateType { INITIALIZATION, MOTION_EXECUTE };

class State
{
	public:
		State();
		State(const State &state);
		~State();

		struct RobotSets{
			RobotSets();
			RobotSets(const RobotSets & robotSets);
			~RobotSets();

			int firstSetCount;
			int secondSetCount;
			lib::ROBOT_ENUM *firstSet;
			lib::ROBOT_ENUM *secondSet;
		};

		void setStateID(const char *stateID);
		const char * getStateID() const;

		void setNumArgument(const char *time);
		int getNumArgument() const;

		void setType(const char *type);
		const char * getType() const;

		void setRobot(const char *robot);
		lib::ROBOT_ENUM getRobot() const;

		void setGeneratorType(std::string genType);
		ecp_mp::task::STATE_MACHINE_ECP_STATES getGeneratorType() const;

		void setStringArgument(const char *trajFilePath);
		const char * getStringArgument() const;

		void setTransition(const char *cond, const char *target, lib::configurator &_config);
		void setProperTransitionResult(bool result);

		const char *returnNextStateID(StateHeap &sh);
		std::list<Transition> * getTransitions() const;

		void showStateContent() const;

		RobotSets *robotSet;

	private:
		int numArgument;
		char *id;
		char *type;
		lib::ROBOT_ENUM robot;
		ecp_mp::task::STATE_MACHINE_ECP_STATES generatorType;
		char *stringArgument;
		std::list<Transition> *stateTransitions;

};


} // namespace common
} // namespace mp
} // namespace mrrocpp


#endif /* _STATE_H_ */
