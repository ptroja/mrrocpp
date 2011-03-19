// ----------------------------------------------------------------------
// Definicje klas wykorzystywanych przy implementacji
// automatu skonczonego
// Autor: Marek Kisiel
// ----------------------------------------------------------------------

#if !defined(_STATE_H_)
#define _STATE_H_

#include <list>

#include "ecp_mp_t_fsautomat.h"
#include "base/lib/impconst.h"
#include "Transition.h"

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

	struct RobotSets
	{
		RobotSets();
		RobotSets(const RobotSets & robotSets);
		~RobotSets();

		int firstSetCount;
		int secondSetCount;
		lib::robot_name_t *firstSet;
		lib::robot_name_t *secondSet;
	};

	void setStateID(const std::string & stateID);
	const char * getStateID() const;

	void setNumArgument(const char *time);
	int getNumArgument() const;

	void setType(const std::string & _type);
	const char * getType() const;

	void setRobot(const std::string & _robot);
	lib::robot_name_t getRobot() const;

	void setGeneratorType(const std::string & genType);
	std::string getGeneratorType() const;

	void setStringArgument(const std::string & trajFilePath);
	const char * getStringArgument() const;

	void setTransition(const char *cond, const char *target, lib::configurator &_config);
	void setProperTransitionResult(bool result);

	const char *returnNextStateID(StateHeap &sh);
	std::list <Transition> * getTransitions() const;

	void showStateContent() const;

	RobotSets *robotSet;

private:
	int numArgument;
	std::string id;
	std::string type;
	lib::robot_name_t robot;
	std::string generatorType;
	std::string stringArgument;

	std::list <Transition> *stateTransitions;
};

} // namespace common
} // namespace mp
} // namespace mrrocpp


#endif /* _STATE_H_ */
