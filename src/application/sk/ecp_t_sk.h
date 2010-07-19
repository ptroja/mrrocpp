#if !defined(_ECP_T_SK_H)
#define _ECP_T_SK_H

#include "base/ecp/ecp_task.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {


/** @defgroup edge_following Edge following
 *  @ingroup application
 *  Single robot application of IRp6 manipulator following of an uknown contour
 *  @{
 */

class sk: public common::task::task  {
protected:
	generator::tff_nose_run* nrg;
	generator::y_edge_follow_force* yefg;
	generator::bias_edp_force* befg;
	bool save_activated;

public:
	// KONSTRUKTORY
	sk(lib::configurator &_config);

	// methods for ECP template to redefine in concrete classes
	void main_task_algorithm(void);
};

/** @} */ // end of edge_following

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif
