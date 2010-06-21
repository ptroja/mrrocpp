// -------------------------------------------------------------------------
//                            ecp_st_go.h
// -------------------------------------------------------------------------

#if !defined(_ECP_ST_BIAS_EDP_FORCE_H)
#define _ECP_ST_BIAS_EDP_FORCE_H

#include "ecp/ecp_task.h"
#include "ecp_mp/task/ecp_mp_st_bias_edp_force.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {
class bias_edp_force;
}

namespace task {

class ecp_sub_task_bias_edp_force : public ecp_sub_task
{

private:
	generator::bias_edp_force* befg;
public:

	ecp_sub_task_bias_edp_force(task &_ecp_t);

	void conditional_execution();
};

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp


#endif
