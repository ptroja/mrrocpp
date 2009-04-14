// -------------------------------------------------------------------------
//                            ecp.h dla QNX6
// Deklaracje struktur danych i metod dla procesow ECP 
// 
// -------------------------------------------------------------------------

#if !defined(_ECP_GEN_PAWEL_H)
#define _ECP_GEN_PAWEL_H

#include "lib/impconst.h"
#include "lib/com_buf.h"
#include "lib/typedefs.h"
#include "ecp/common/ecp_generator.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {


// --------------------------------------------------------------------------

class pawel : public common::generator::base {

protected:

	int state;
	int step_no;	
//    	double start_joint_arm_coordinates[MAX_SERVOS_NR];
	
public:

	lib::trajectory_description td;
	
	// konstruktor
	~pawel();
	pawel(common::task::base& _ecp_task, int step);  
	
	virtual bool first_step();
	virtual bool next_step();

}; // end:
// --------------------------------------------------------------------------

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif
