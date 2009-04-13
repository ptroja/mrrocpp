// -------------------------------------------------------------------------
//                            ecp_g_test.h
// Deklaracje struktur danych i metod dla procesow ECP z wykorzystaniem sily
// 
// -------------------------------------------------------------------------

#if !defined(_ECP_GEN_TEST_IRP6P_H)
#define _ECP_GEN_TEST_IRP6P_H

#include "common/impconst.h"		// frame_tab
#include "common/com_buf.h"			// trajectory_description

#include "ecp/common/ecp_generator.h"

namespace mrrocpp {
namespace ecp {
namespace irp6p {
namespace generator {

// --------------------------------------------------------------------------
// Generator trajektorii prostoliniowej dla zadan yoyka z wodzeniem za nos
class y_simple_generator : public common::generator::ecp_generator {

protected:
	long run_counter;
	bool second_step;
	frame_tab previous_frame;
	
public:	
	trajectory_description td;
	int step_no;
	double delta[6];
	
	// konstruktor
	y_simple_generator(common::task::ecp_task& _ecp_task, int step=0);
	
	virtual bool first_step ();

	virtual bool next_step ();

}; // end:
// --------------------------------------------------------------------------

}
} // namespace irp6p
} // namespace ecp
} // namespace mrrocpp

#endif
