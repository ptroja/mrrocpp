// -------------------------------------------------------------------------
//                            ecp_g_pjg.h dla QNX6
// Deklaracje struktur danych i metod dla procesow ECP 
// 
// -------------------------------------------------------------------------

#if !defined(_ECP_GEN_PLAYERJOY_H)
#define _ECP_GEN_PLAYERJOY_H

#include "common/impconst.h"

#include "ecp/common/ecp_generator.h"

namespace mrrocpp {
namespace ecp {
namespace common {

// --------------------------------------------------------------------------
// Generator trajektorii prostoliniowej 
class playerjoy_generator : public common::ecp_generator 
{

protected:

	long run_counter;
	bool second_step;
    double start_joint_arm_coordinates[MAX_SERVOS_NR];
	
public:
	//trajectory_description td;
	int step_no;
	
	// konstruktor
	playerjoy_generator(ecp_task& _ecp_task, int step=0);  
	
	virtual bool first_step ();

	virtual bool next_step ();

}; // end:
// --------------------------------------------------------------------------

} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif /* _ECP_GEN_PLAYERJOY_H */
