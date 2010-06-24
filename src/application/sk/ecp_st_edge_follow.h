// -------------------------------------------------------------------------
//                            ecp_st_go.h
// -------------------------------------------------------------------------

#if !defined(_ECP_SUB_TASK_EDGE_FOLLOW_H)
#define _ECP_SUB_TASK_EDGE_FOLLOW_H

#include "base/ecp/ecp_task.h"
#include "application/sk/ecp_mp_st_edge_follow.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {
class y_edge_follow_force;
}

namespace task {

class ecp_sub_task_edge_follow : public ecp_sub_task
{

private:
	generator::y_edge_follow_force* yefg;

public:
	ecp_sub_task_edge_follow(task &_ecp_t);

	void conditional_execution();
};

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp


#endif
