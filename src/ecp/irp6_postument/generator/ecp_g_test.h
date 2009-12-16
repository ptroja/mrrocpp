// -------------------------------------------------------------------------
//                            generator/ecp_g_test.h
// Deklaracje struktur danych i metod dla procesow ECP z wykorzystaniem sily
// 
// -------------------------------------------------------------------------

#if !defined(_ECP_GEN_TEST_IRP6P_H)
#define _ECP_GEN_TEST_IRP6P_H

#include "lib/impconst.h"		// lib::frame_tab
#include "lib/com_buf.h"			// lib::trajectory_description

#include "ecp/common/generator/ecp_generator.h"

namespace mrrocpp {
namespace ecp {
namespace irp6p {
namespace generator {

// --------------------------------------------------------------------------
// Generator trajektorii prostoliniowej dla zadan yoyka z wodzeniem za nos
class y_simple : public common::generator::generator {

protected:
	long run_counter;
	bool second_step;
	lib::frame_tab previous_frame;
	
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
} // namespace irp6p
} // namespace ecp
} // namespace mrrocpp

#endif
