#if !defined(_ECP_ST_TFF_NOSE_RUN_H)
#define _ECP_ST_TFF_NOSE_RUN_H

/*!
 * @file
 * @brief File contains tff_nose_run declaration
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup subtasks
 */

#include "base/ecp/ecp_sub_task.h"
#include "subtask/ecp_mp_st_tff_nose_run.h"
#include "generator/ecp/force/ecp_g_tff_nose_run.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {
class tff_nose_run;
}

namespace sub_task {

/*!
 * @brief subtask to execute tff_nose_run generator
 *
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 * @ingroup subtasks
 */
class tff_nose_run : public sub_task
{

private:

public:

	/*!
	 * @brief tff_nose_run generator pojnter
	 */
	generator::tff_nose_run* nrg;

	/**
	 * @brief Constructor
	 * @param _ecp_t ecp task object reference.
	 */
	tff_nose_run(task::task &_ecp_t);

	void conditional_execution();
};

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp


#endif
