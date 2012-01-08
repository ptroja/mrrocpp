/*!
 * @file
 * @brief File contains mp robot class definition for SwarmItFix Head
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup sbench
 */

#include "mp_r_sbench.h"
#include "robot/sbench/const_sbench.h"

namespace mrrocpp {
namespace mp {
namespace robot {

sbench::sbench(task::task &mp_object_l) :
		mp::robot::robot(lib::sbench::ROBOT_NAME, mp_object_l, lib::sbench::NUM_OF_SERVOS)
{
}

} // namespace robot
} // namespace mp
} // namespace mrrocpp

