#if !defined(_ECP_SUB_TASK_SPRING_CONTACT_H)
#define _ECP_SUB_TASK_SPRING_CONTACT_H

/*!
 * @file
 * @brief File contains edge_follow sub_task class declaration of unknown contour following application.
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup edge_follow
 */

#include "base/ecp/ecp_sub_task.h"
#include "ecp_mp_st_spring_contact.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {
class spring_contact;
}

namespace sub_task {

/*!
 * @brief subtask to execute y_edge_follow_force generator
 *
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 * @ingroup subtasks edge_follow
 */
class spring_contact : public sub_task
{

private:

	/*!
	 * @brief y_edge_follow_force generator pointer
	 */
	generator::spring_contact* scg;

public:

	/**
	 * @brief Constructor
	 * @param _ecp_t ecp task object reference.
	 */
	spring_contact(task::task &_ecp_t);

	void conditional_execution();
};

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif
