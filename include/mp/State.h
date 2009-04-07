// ----------------------------------------------------------------------
// Definicje klas wykorzystywanych przy implementacji
// automatu skonczonego
// Autor: Marek Kisiel
// ----------------------------------------------------------------------

#if !defined(_STATE_H_)
#define _STATE_H_

#include <list>

#include "ecp_mp/ecp_mp_t_fsautomat.h"
#include "common/impconst.h"
#include "mp/Transition.h"


using namespace ecp_mp;

//enum StateType { INITIALIZATION, MOTION_EXECUTE };

class State
{
	public:
		State();
		State(const State &state);
		~State();

		typedef struct RobotSets{
			RobotSets();
			RobotSets(const RobotSets & robotSets);
			~RobotSets();
			
			int firstSetCount;
			int secondSetCount;
			ROBOT_ENUM *firstSet;
			ROBOT_ENUM *secondSet;
		};
		static ROBOT_ENUM returnProperRobot(char * robotName);
		void setStateID(char *stateID);
		char * getStateID() const;
		void setNumArgument(char *time);
		int getNumArgument() const;
		void setType(char *type);
		char *  getType() const;
		void setRobot(char *robot);
		ROBOT_ENUM getRobot() const;
		void setGeneratorType(char *genType);
		ecp_mp::task::STATE_MACHINE_ECP_STATES getGeneratorType() const;
		void setStringArgument(char *trajFilePath);
		char *  getStringArgument() const;
		void setTransition(char *cond, char *target, configurator &_config);
		void setProperTransitionResult(bool result);
		const char *returnNextStateID(StateHeap &sh);
		std::list<Transition> * getTransitions() const;

		void showStateContent() const;
		
		RobotSets *robotSet;
		
	private:
		int numArgument;
		char *id;
		char *type;
		ROBOT_ENUM robot;
		ecp_mp::task::STATE_MACHINE_ECP_STATES generatorType;
		char *stringArgument;
		std::list<Transition> *stateTransitions;
	
};


#endif /* _STATE_H_ */
