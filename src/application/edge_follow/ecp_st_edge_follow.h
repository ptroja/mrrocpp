#if !defined(_ECP_SUB_task_EDGE_FOLLOW_H)
#define _ECP_SUB_task_EDGE_FOLLOW_H

/*!
 * @file ecp_st_edge_follow.h
 * @brief File contains edge_follow sub_task class declaration of unknown contour following application.
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup edge_follow
 */

#include "base/ecp/ecp_task.h"
#include "application/edge_follow/ecp_mp_st_edge_follow.h"

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
