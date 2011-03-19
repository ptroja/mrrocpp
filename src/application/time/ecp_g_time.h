// -------------------------------------------------------------------------
//                            ecp.h dla QNX6
// Deklaracje struktur danych i metod dla procesow ECP 
// 
// -------------------------------------------------------------------------

//   tdes.arm_type = lib::XYZ_EULER_ZYZ;

#if !defined(_ECP_GEN_PLOT_H)
#define _ECP_GEN_PLOT_H

#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"

#include "base/ecp/ecp_generator.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

// --------------------------------------------------------------------------
// Generator trajektorii prostoliniowej 
class time : public common::generator::generator 
{

protected:

	long run_counter;
	bool second_step;
    double start_joint_arm_coordinates[lib::MAX_SERVOS_NR];
	
public:
	lib::trajectory_description td;
	int step_no;
	
	// konstruktor
	time(common::task::task& _ecp_task, int step=0);  
	
	virtual bool first_step ();

	virtual bool next_step ();

};

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif
