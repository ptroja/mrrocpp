#if !defined(_ECP_ST_BIAS_EDP_FORCE_H)
#define _ECP_ST_BIAS_EDP_FORCE_H

/*!
 * @file
 * @brief File contains ecp_sub_task_bias_edp_force declaration
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup subtasks
 */

#include "base/ecp/ecp_task.h"
#include "subtask/ecp_mp_st_bias_edp_force.h"

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
