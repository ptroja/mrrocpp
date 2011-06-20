/*!
 * @file
 * @brief File contains mp robot class definition for SwarmItFix Mobile Base
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup smb
 */

#include "mp_r_smb1.h"

namespace mrrocpp {
namespace mp {
namespace robot {

smb1::smb1(task::task &mp_object_l) :
	smb(lib::smb1::ROBOT_NAME, mp_object_l)
{
}

} // namespace robot
} // namespace mp
} // namespace mrrocpp

