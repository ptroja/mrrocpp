/*!
 * @file
 * @brief File contains tff_nose_run definition
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup subtasks
 */

#include "base/lib/typedefs.h"
#include "base/lib/sr/srlib.h"

#include "subtask/ecp_st_tff_nose_run.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace sub_task {

tff_nose_run::tff_nose_run(task::task &_ecp_t) :
	sub_task(_ecp_t)
{
	nrg = new generator::tff_nose_run(_ecp_t, 8);
}

void tff_nose_run::conditional_execution()
{

	nrg->Move();
}

} // namespace task

} // namespace common
} // namespace ecp
} // namespace mrrocpp
