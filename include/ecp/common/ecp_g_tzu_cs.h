#if !defined(_ECP_GEN_TZU_CS_H)
#define _ECP_GEN_TZU_CS_H

#include "common/impconst.h"		// frame_tab
#include "common/com_buf.h"		// trajectory_description

#include "ecp/common/ecp_generator.h"

class tzu_simple_generator : public ecp_generator 
{
protected:
	bool first_run;
	bool finished;
	
public:
	trajectory_description td;
	// zobaczyc co to jest ten step_no
	int step_no;
	
	int move_status;
	double current_position[6];	
	double new_position[6];
	
	static const int MOVE_END = -1;
	static const int MOVE_PHASE_ONE = 1;
	static const int MOVE_PHASE_TWO = 2;
	static const int MOVE_PHASE_THREE = 3;
	tzu_simple_generator(ecp_task& _ecp_task, int step=0);
	
	virtual bool first_step ();
	virtual bool next_step ();
}; 

#endif

