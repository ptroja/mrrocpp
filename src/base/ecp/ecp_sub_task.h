#if !defined(_ECP_SUB_TASK_H)
#define _ECP_SUB_TASK_H

/*!
 * @file
 * @brief File contains ecp base sub_task declaration
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup ecp
 */

#include "base/ecp/ecp_robot.h"
#include "base/ecp/ecp_task.h"
#include "base/ecp_mp/ecp_mp_task.h"

namespace mrrocpp {
namespace ecp {
namespace common {
//namespace robot {
//class ecp_robot;
//}
//namespace task {
//class task;
//}

namespace sub_task {
/*!
 * @brief Base class of all ecp sub tasks
 *
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 * @ingroup ecp
 */
class sub_task_base {
public:
	/**
	 * @brief method to implement in derived classes that stores main sub task algorithm
	 */
	virtual void conditional_execution() = 0;
};

template<typename ECP_ROBOT_T = robot::ecp_robot>
class _sub_task : public sub_task_base
{
protected:
	/**
	 * @brief ecp task object reference
	 */
	task::_task<ECP_ROBOT_T> &ecp_t;

	/**
	 * @brief the reference to sr communication object in multi thread version
	 */
	lib::sr_ecp& sr_ecp_msg;

public:
	/**
	 * @brief Constructor
	 * @param _ecp_t ecp task object reference.
	 */
	_sub_task(task::_task<ECP_ROBOT_T> &_ecp_t) :
		ecp_t(_ecp_t), sr_ecp_msg(*(_ecp_t.sr_ecp_msg))
	{}

	typedef ECP_ROBOT_T robot_t;
};

typedef _sub_task<> sub_task;

} // namespace sub_task
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif /* _ECP_TASK_H */
