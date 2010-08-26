#if !defined(_ECP_T_edge_follow_MR_H)
#define _ECP_T_edge_follow_MR_H

/*!
 * @file ecp_t_edge_follow_mr.h
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

class edge_follow_mr : public common::task::task
{
protected:

public:
	// KONSTRUKTORY
	edge_follow_mr(lib::configurator &_config);

};

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif
