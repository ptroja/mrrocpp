// -------------------------------------------------------------------------
//                            ecp.h dla QNX6
// Deklaracje struktur danych i metod dla procesow ECP 
// 
// -------------------------------------------------------------------------


#if !defined(_ECP_GEN_PP_H)
#define _ECP_GEN_PP_H

#include "common/impconst.h"
#include "common/com_buf.h"
#include "lib/mathtr.h"
#include "ecp/common/ecp_generator.h"


// --------------------------------------------------------------------------
// Generator trajektorii prostoliniowej 
class progpanel_generator : public ecp_generator 
{

protected:
	int node_counter;
	long run_counter;
	bool second_step;
    double start_joint_arm_coordinates[MAX_SERVOS_NR];
	
public:
	trajectory_description td;
	int step_no;
	
	// konstruktor
	progpanel_generator();
	progpanel_generator(ecp_task& _ecp_task, int step=0);  
	
	virtual bool first_step ();
	virtual bool next_step ();

}; // end:
// --------------------------------------------------------------------------


#endif
