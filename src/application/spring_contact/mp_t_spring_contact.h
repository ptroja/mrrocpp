#if !defined(__MP_T_SPRING_CONTACT_H)
#define __MP_T_SPRING_CONTACT_H

/*!
 * @file
 * @brief File contains mp_task class declaration
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup spring_contact
 */

#include "base/mp/mp_task.h"

namespace mrrocpp {
namespace mp {
namespace task {

/*!
 * @brief Task that executes motion of manipulator and its gripper to follow an unknown contour
 *
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 * @ingroup edge_follow
 */
class spring_contact : public task
{
protected:

public:

	/**
	 * @brief Constructor
	 * @param _config configurator object reference.
	 */
	spring_contact(lib::configurator &_config);

	void create_robots(void);

	void main_task_algorithm(void);

};

} // namespace task
} // namespace mp
} // namespace mrrocpp

#endif
