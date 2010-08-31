/*!
 * @file
 * @brief File contains mp robot class definition for Sarkofag
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup sarkofag
 */

#include "robot/sarkofag/mp_r_sarkofag.h"
#include "robot/sarkofag/const_sarkofag.h"

namespace mrrocpp {
namespace mp {
namespace robot {

sarkofag::sarkofag(task::task &mp_object_l) :
	motor_driven(lib::sarkofag::ROBOT_SARKOFAG, lib::sarkofag::ECP_SECTION, mp_object_l, lib::sarkofag::SARKOFAG_NUM_OF_SERVOS)
{
}

} // namespace robot
} // namespace mp
} // namespace mrrocpp
