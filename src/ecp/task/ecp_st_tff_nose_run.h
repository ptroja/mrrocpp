// -------------------------------------------------------------------------
//                            ecp_st_go.h
// -------------------------------------------------------------------------

#if !defined(_ECP_ST_TFF_NOSE_RUN_H)
#define _ECP_ST_TFF_NOSE_RUN_H

#include "ecp/task/ecp_task.h"
#include "ecp_mp/task/ecp_mp_st_tff_nose_run.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {
class tff_nose_run;
}

namespace task {

class ecp_sub_task_tff_nose_run : public ecp_sub_task
{

private:

public:
	generator::tff_nose_run* nrg;
	ecp_sub_task_tff_nose_run(task &_ecp_t);

	void conditional_execution();
};

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp


#endif
