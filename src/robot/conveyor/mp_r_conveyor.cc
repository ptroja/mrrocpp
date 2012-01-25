/*!
 * @file
 * @brief File contains mp robot class definition for Conveyor
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup conveyor
 */

#include "robot/conveyor/mp_r_conveyor.h"

namespace mrrocpp {
namespace mp {
namespace robot {

conveyor::conveyor(task::task &mp_object_l) :
		mp::robot::robot(lib::conveyor::ROBOT_NAME, mp_object_l, lib::conveyor::NUM_OF_SERVOS)
{
}

} // namespace robot
} // namespace mp
} // namespace mrrocpp

