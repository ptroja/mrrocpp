// -------------------------------------------------------------------------
//                            ecp.h dla QNX6
// Deklaracje struktur danych i metod dla procesow ECP 
// 
// -------------------------------------------------------------------------

//   tdes.arm_type = XYZ_EULER_ZYZ;

#if !defined(_ECP_GEN_PLOT_H)
#define _ECP_GEN_PLOT_H

#include "common/impconst.h"
#include "common/com_buf.h"

#include "ecp/common/ecp_generator.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

// --------------------------------------------------------------------------
// Generator trajektorii prostoliniowej 
class time_generator : public common::generator::ecp_generator 
{

protected:

	long run_counter;
	bool second_step;
    double start_joint_arm_coordinates[MAX_SERVOS_NR];
	
public:
	trajectory_description td;
	int step_no;
	
	// konstruktor
	time_generator(common::task::ecp_task& _ecp_task, int step=0);  
	
	virtual bool first_step ();

	virtual bool next_step ();

};

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif
