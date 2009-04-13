// -------------------------------------------------------------------------
//                            ecp_g_plot.h dla QNX6
// Deklaracje struktur danych i metod dla procesow ECP 
// 
// -------------------------------------------------------------------------

#if !defined(_ECP_GEN_PLOT_H)
#define _ECP_GEN_PLOT_H

#include "common/impconst.h"	// frame_tab
#include "common/com_buf.h"		// trajectory_description

#include "ecp/common/ecp_generator.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

// --------------------------------------------------------------------------
// Generator trajektorii prostoliniowej 
class y_simple_generator : public common::generator::ecp_generator 
{

protected:

	long run_counter;
	bool second_step;
	frame_tab previous_frame;
	bool finished;
	
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
	y_simple_generator(common::task::ecp_task& _ecp_task, int step=0);
	
	virtual bool first_step ();
	virtual bool next_step ();

}; // end:
// --------------------------------------------------------------------------

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif
