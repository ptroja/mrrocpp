#if !defined(_ECP_SUB_TASK_H)
#define _ECP_SUB_TASK_H

/*!
 * @file
 * @brief File contains ecp base sub_task declaration
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup ecp
 */

#include "base/ecp_mp/ecp_mp_task.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace robot {
class ecp_robot;
}
namespace task {

class task;

}

namespace sub_task {
/*!
 * @brief Base class of all ecp sub tasks
 *
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 * @ingroup ecp
 */
class sub_task
{
protected:

	/**
	 * @brief ecp task object reference
	 */
	task::task &ecp_t;

	/**
	 * @brief the reference to sr communication object in multi thread version
	 */
	lib::sr_ecp& sr_ecp_msg;

public:

	/**
	 * @brief Constructor
	 * @param _ecp_t ecp task object reference.
	 */
	sub_task(task::task &_ecp_t);

	/**
	 * @brief method to implement in derived classes that stores main sub task algorithm
	 */
	virtual void conditional_execution() = 0;
};

} // namespace sub_task
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif /* _ECP_TASK_H */
