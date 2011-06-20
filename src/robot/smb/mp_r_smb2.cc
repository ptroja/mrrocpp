/*!
 * @file
 * @brief File contains mp robot class definition for SwarmItFix Mobile Base
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup smb
 */

#include "mp_r_smb2.h"

namespace mrrocpp {
namespace mp {
namespace robot {

smb2::smb2(task::task &mp_object_l) :
	smb(lib::smb2::ROBOT_NAME, mp_object_l)
{
}

} // namespace robot
} // namespace mp
} // namespace mrrocpp

