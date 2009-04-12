// -------------------------------------------------------------------------
//                            ecp.h dla QNX6
// Deklaracje struktur danych i metod dla procesow ECP 
// 
// -------------------------------------------------------------------------

#if !defined(_ECP_GEN_PAWEL_H)
#define _ECP_GEN_PAWEL_H

#include "common/impconst.h"
#include "common/com_buf.h"
#include "common/typedefs.h"
#include "ecp/common/ecp_generator.h"

namespace mrrocpp {
namespace ecp {
namespace common {


// --------------------------------------------------------------------------

class pawel_generator : public common::ecp_generator {

protected:

	int state;
	int step_no;	
//    	double start_joint_arm_coordinates[MAX_SERVOS_NR];
	
public:

	trajectory_description td;
	
	// konstruktor
	~pawel_generator();
	pawel_generator(common::task::ecp_task& _ecp_task, int step);  
	
	virtual bool first_step();
	virtual bool next_step();

}; // end:
// --------------------------------------------------------------------------

} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif
