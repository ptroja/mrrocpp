/*!
 * @file
 * @brief File contains ecp_sub_task_tff_nose_run definition
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup subtasks
 */

#include "base/lib/typedefs.h"
#include "base/lib/sr/srlib.h"
#include "generator/ecp/ecp_g_force.h"
#include "subtask/ecp_st_tff_nose_run.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

ecp_sub_task_tff_nose_run::ecp_sub_task_tff_nose_run(task &_ecp_t) :
	ecp_sub_task(_ecp_t)
{
	nrg = new generator::tff_nose_run(_ecp_t, 8);
}

void ecp_sub_task_tff_nose_run::conditional_execution()
{

	nrg->Move();
}

} // namespace task

} // namespace common
} // namespace ecp
} // namespace mrrocpp
