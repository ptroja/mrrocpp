/*!
 * @file ecp_st_edge_follow.cc
 * @brief File contains edge_follow sub_task class definition of unknown contour following application.
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup edge_follow
 */

#include "application/edge_follow/ecp_g_edge_follow.h"
#include "application/edge_follow/ecp_st_edge_follow.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

ecp_sub_task_edge_follow::ecp_sub_task_edge_follow(task &_ecp_t) :
	ecp_sub_task(_ecp_t)
{
	yefg = new generator::y_edge_follow_force(_ecp_t, 8);
}

void ecp_sub_task_edge_follow::conditional_execution()
{

	yefg->Move();
}

} // namespace task

} // namespace common
} // namespace ecp
} // namespace mrrocpp
