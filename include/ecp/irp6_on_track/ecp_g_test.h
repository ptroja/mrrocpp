// -------------------------------------------------------------------------
//                            ecp.h dla QNX6
// Deklaracje struktur danych i metod dla procesow ECP z wykrozsytsaniem sily
// 
// -------------------------------------------------------------------------

//   tdes.arm_type = XYZ_EULER_ZYZ;

#if !defined(_ECP_GEN_TEST_IRP6OT_H)
#define _ECP_GEN_TEST_IRP6OT_H

#include "common/impconst.h"
#include "common/com_buf.h"

#include "ecp/common/ecp_generator.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot {

// --------------------------------------------------------------------------
// Generator trajektorii prostoliniowej dla zadan yoyka z wodzeniem za nos
class y_simple_generator : public common::ecp_generator {

protected:
	long run_counter;
	bool second_step;
	frame_tab previous_frame;
	
public:	
	trajectory_description td;
	int step_no;
	double delta[6];
	double frame1[4][4];
	int valid_measure;
	bool the_first;
	
	double alfa;
	double beta;
	double gammax;
	
	// konstruktor
	y_simple_generator(ecp_task& _ecp_task, int step=0);  
	virtual ~y_simple_generator() {};

	virtual bool first_step ();

	virtual bool next_step ();

}; // end:
// --------------------------------------------------------------------------

} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp

#endif
