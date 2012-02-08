#if !defined(_ECP_T_RCSC_H)
#define _ECP_T_RCSC_H

#include <boost/shared_ptr.hpp>

#include "robot/irp6ot_m/ecp_r_irp6ot_m.h"
#include "robot/irp6p_m/ecp_r_irp6p_m.h"

#include "base/ecp/ecp_task.h"
#include "base/ecp/ecp_g_transparent.h"
#include "generator/ecp/force/ecp_g_weight_measure.h"

#include "generator/ecp/force/ecp_g_tff_rubik_face_rotate.h"
#include "generator/ecp/tff_gripper_approach/ecp_g_tff_gripper_approach.h"
#include "generator/ecp/ecp_g_newsmooth.h"

using boost::shared_ptr;

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

class rcsc : public common::task::task
{
protected:
	//generatory
	generator::transparent* gt;


	generator::tff_rubik_face_rotate* rfrg;

public:
	rcsc(lib::configurator &_config);
	virtual ~rcsc();

	// methods for ECP template to redefine in concrete classes
	void mp_2_ecp_next_state_string_handler(void);
};

}
} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp

#endif
