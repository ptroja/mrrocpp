#if !defined(_ECP_SUBTASK_H)
#define _ECP_SUBTASK_H

/*!
 * @file
 * @brief File contains ecp base subtask declaration
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup ecp
 */

#include "base/ecp/ecp_robot.h"
#include "base/ecp/ecp_task.h"
#include "base/ecp_mp/ecp_mp_task.h"
#include "ecp_subtask_generator_base.h"

namespace mrrocpp {
namespace ecp {
namespace common {
//namespace robot {
//class ecp_robot;
//}
//namespace task {
//class task;
//}

namespace subtask {
/*!
 * @brief Base class of all ecp sub tasks
 *
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 * @ingroup ecp
 */

template <typename ECP_ROBOT_T = robot::ecp_robot>
class _subtask : public subtask_generator_base
{
protected:
	/**
	 * @brief ecp task object reference
	 */
	task::_task <ECP_ROBOT_T> &ecp_t;

	/**
	 * @brief the reference to sr communication object in multi thread version
	 */
	lib::sr_ecp& sr_ecp_msg;

public:
	/**
	 * @brief Constructor
	 * @param _ecp_t ecp task object reference.
	 */
	_subtask(task::_task <ECP_ROBOT_T> &_ecp_t) :
			subtask_generator_base(), ecp_t(_ecp_t), sr_ecp_msg(*(_ecp_t.sr_ecp_msg))
	{
	}

	typedef ECP_ROBOT_T robot_t;
};

typedef _subtask <> subtask;

} // namespace subtask
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif /* _ECP_TASK_H */
