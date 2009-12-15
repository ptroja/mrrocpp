// -------------------------------------------------------------------------
//                            ecp.h dla QNX6
// Deklaracje struktur danych i metod dla procesow ECP z wykrozsytsaniem sily
// 
// -------------------------------------------------------------------------

#if !defined(_ECP_GEN_TEST_CONV_H)
#define _ECP_GEN_TEST_CONV_H

#include "lib/com_buf.h"		// lib::trajectory_description

#include "ecp/common/generator/ecp_generator.h"

namespace mrrocpp {
namespace ecp {
namespace conveyor {
namespace generator {

// --------------------------------------------------------------------------
// Generator trajektorii prostoliniowej dla zadan yoyka z wodzeniem za nos
class y_simple : public common::generator::generator {

protected:

	
public:	
	lib::trajectory_description td;
	int step_no;
	double delta[6];
	
	// konstruktor
	y_simple(common::task::task& _ecp_task, int step=0);  
	
	virtual bool first_step ();

	virtual bool next_step ();

}; // end:
// --------------------------------------------------------------------------

}
} // namespace conveyor
} // namespace ecp
} // namespace mrrocpp

#endif
