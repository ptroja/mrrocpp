#if !defined(_ECP_SUB_task_EDGE_FOLLOW_H)
#define _ECP_SUB_task_EDGE_FOLLOW_H

/*!
 * @file
 * @brief File contains edge_follow sub_task class declaration of unknown contour following application.
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup edge_follow
 */

#include "base/ecp/ecp_sub_task.h"
#include "application/edge_follow/ecp_mp_st_edge_follow.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {
class y_edge_follow_force;
}

namespace sub_task {

class edge_follow : public sub_task
{

private:
	generator::y_edge_follow_force* yefg;

public:
	edge_follow(task::task &_ecp_t);

	void conditional_execution();
};

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp


#endif
