/*!
 * @file
 * @brief File contains mp robot class definition for SwarmItFix Mobile Base
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup smb
 */

#include "robot/smb/mp_r_smb.h"


namespace mrrocpp {
namespace mp {
namespace robot {

smb::smb(task::task &mp_object_l) :
	robot(lib::smb::ROBOT_NAME, lib::smb::ECP_SECTION, mp_object_l, lib::smb::NUM_OF_SERVOS)
{
}

} // namespace robot
} // namespace mp
} // namespace mrrocpp

