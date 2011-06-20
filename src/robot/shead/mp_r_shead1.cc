/*!
 * @file
 * @brief File contains mp robot class definition for SwarmItFix Head
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup shead
 */

#include "mp_r_shead1.h"

namespace mrrocpp {
namespace mp {
namespace robot {

shead1::shead1(task::task &mp_object_l) :
	shead(lib::shead1::ROBOT_NAME, mp_object_l)
{
}

} // namespace robot
} // namespace mp
} // namespace mrrocpp
