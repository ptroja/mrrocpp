#if !defined(MP_R_SBENCH_H_)
#define MP_R_SBENCH_H_

/*!
 * @file
 * @brief File contains mp robot class declaration for SwarmItFix Head
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup sbench
 */

#include "base/mp/mp_robot.h"
#include "robot/sbench/const_sbench.h"

namespace mrrocpp {
namespace mp {
namespace robot {

/*!
 * @brief SwarmItFix Head mp robot class
 *
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 * @ingroup sbench
 */
class sbench : public robot
{
public:
	/**
	 * @brief constructor
	 * @param mp_object_l mp task object reference
	 */
	sbench(const lib::robot_name_t & l_robot_name, task::task &mp_object_l);
};

} // namespace robot
} // namespace mp
} // namespace mrrocpp
#endif /*MP_R_SBENCH_H_*/
