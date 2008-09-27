// ----------------------------------------------------------------------
// Definicje klas wykorzystywanych przy implementacji
// automatu skonczonego
// Autor: Marek Kisiel
// ----------------------------------------------------------------------

#if !defined(_STATE_H_)
#define _STATE_H_

#include "ecp_mp/ecp_mp_t_fsautomat.h"
#include "common/impconst.h"

//enum StateType { INITIALIZATION, MOTION_EXECUTE };

class State
{
	public:
		State();
		State(const State &state);
		~State();
		
		void setName(char *name);
		char * getName() const;
		void setType(char *type);
		char *  getType() const;
		void setRobot(char *robot);
		ROBOT_ENUM getRobot() const;
		void setGeneratorType(char *genType);
		STATE_MACHINE_ECP_STATES getGeneratorType() const;
		void setTrajectoryFilePath(char *trajFilePath);
		char *  getTrajectoryFilePath() const;

		void showStateContent() const;
		
	private:
		char *name;
		char *type;
		ROBOT_ENUM robot;
		STATE_MACHINE_ECP_STATES generatorType;
		char *trajectoryFilePath;
	
};


#endif /* _STATE_H_ */
