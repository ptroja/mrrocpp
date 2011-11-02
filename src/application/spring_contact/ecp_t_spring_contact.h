#if !defined(_ECP_T_SPRING_CONTACT_H)
#define _ECP_T_SPRING_CONTACT_H

/*!
 * @file
 * @brief File containsecp_task class declaration
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup spring_contact
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
class spring_contact : public common::task::task
{
protected:

public:

	/**
	 * @brief Constructor
	 * @param _config configurator object reference.
	 */
	spring_contact(lib::configurator &_config);
};

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif
