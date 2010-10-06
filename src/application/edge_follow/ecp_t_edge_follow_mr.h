#if !defined(_ECP_T_edge_follow_MR_H)
#define _ECP_T_edge_follow_MR_H

/*!
 * @file
 * @brief File contains edge_follow_mr ecp_task class declaration of unknown contour following application.
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup edge_follow
 */

#include "base/ecp/ecp_task.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

/*!
 * @brief task to execute edge_following.
 * it executes two subtasks EDGE_FOLLOW and ECP_ST_BIAS_EDP_FORCE
 *
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 * @ingroup edge_follow
 */
class edge_follow_mr : public common::task::task
{
protected:

public:

	/**
	 * @brief Constructor
	 * @param _config configurator object reference.
	 */
	edge_follow_mr(lib::configurator &_config);
};

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif
