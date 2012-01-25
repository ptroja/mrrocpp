/*!
 * @file
 * @brief File contains tff_nose_run definition
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup subtasks
 */

#include "base/lib/typedefs.h"
#include "base/lib/sr/sr_ecp.h"

#include "ecp_st_tff_nose_run.h"

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
	ecp_mp::sub_task::behaviour_specification_data_type beh;

	switch ((ecp_mp::sub_task::communication_type) ecp_t.mp_command.ecp_next_state.variant)
	{
		case ecp_mp::sub_task::behaviour_specification:
			ecp_t.mp_command.ecp_next_state.sg_buf.get(beh);
			break;
		case ecp_mp::sub_task::no_data:
			break;
		default:
			break;
	}

	nrg->configure_behaviour(beh.behaviour[0], beh.behaviour[1], beh.behaviour[2], beh.behaviour[3], beh.behaviour[4], beh.behaviour[5]);

	nrg->Move();
}

} // namespace task

} // namespace common
} // namespace ecp
} // namespace mrrocpp
