// -------------------------------------------------------------------------
//                            generator/ecp_g_force.h dla QNX6
// Deklaracje generatorow dla procesow ECP z wykorzystaniem sily
//
// -------------------------------------------------------------------------


#if !defined(_ECP_GEN_SK_H)
#define _ECP_GEN_SK_H

#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "ecp/common/generator/ecp_g_teach_in.h"
#include "lib/mrmath/mrmath.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

/** @addtogroup edge_following
 *
 *  @{
 */

class y_edge_follow_force: public teach_in {
protected:

	lib::ECP_POSE_SPECIFICATION emptyps;
	lib::trajectory_description td;
	const int step_no;
	double delta[6];
	lib::Homog_matrix basic_rot_frame;
	lib::Homog_matrix tool_frame;
	lib::Homog_matrix ex_rot_frame;

public:

	// konstruktor
	y_edge_follow_force(common::task::task& _ecp_task, int step);

	virtual bool first_step();

	virtual bool next_step();

}; // end:
///

/** @} */// end of edge_following


} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif
