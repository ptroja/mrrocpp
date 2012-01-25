/*!
 * @file
 * @brief File contains mp robot class definition for SwarmItFix Head
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup shead
 */

#include "mp_r_shead2.h"

namespace mrrocpp {
namespace mp {
namespace robot {

shead2::shead2(task::task &mp_object_l) :
	shead(lib::shead2::ROBOT_NAME, mp_object_l)
{
}

} // namespace robot
} // namespace mp
} // namespace mrrocpp
