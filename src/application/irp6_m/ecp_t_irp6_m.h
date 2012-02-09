#if !defined(_ECP_T_IRP6_M_H)
#define _ECP_T_IRP6_M_H

#include <boost/shared_ptr.hpp>

#include "robot/irp6ot_m/ecp_r_irp6ot_m.h"
#include "robot/irp6p_m/ecp_r_irp6p_m.h"

#include "base/ecp/ecp_task.h"
#include "generator/ecp/transparent/ecp_g_transparent.h"
#include "generator/ecp/weight_measure/ecp_g_weight_measure.h"

#include "generator/ecp/tff_rubik_face_rotate/ecp_g_tff_rubik_face_rotate.h"
#include "generator/ecp/tff_gripper_approach/ecp_g_tff_gripper_approach.h"
#include "generator/ecp/ecp_g_newsmooth.h"

using boost::shared_ptr;

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

class irp6_m : public common::task::task
{
protected:

public:
	irp6_m(lib::configurator &_config);
	virtual ~irp6_m();

};

}
} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp

#endif
