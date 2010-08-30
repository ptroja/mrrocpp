/*!
 * @file
 * @brief File contains ecp_mp base robot definition
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup ecp_mp
 */

#include "base/lib/impconst.h"
#include "base/ecp_mp/ecp_mp_robot.h"

namespace mrrocpp {
namespace ecp_mp {

robot::robot(lib::robot_name_t _robot_name) :
	robot_name(_robot_name)
{
}

} // namespace ecp_mp
} // namespace mrrocpp
