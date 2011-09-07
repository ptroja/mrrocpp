/*!
 * @file
 * @brief File contains ecp robot class definition for SwarmItFix Mobile Base
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup smb
 */

#include "ecp_r_smb2.h"

namespace mrrocpp {
namespace ecp {
namespace smb2 {

robot::robot(lib::configurator &_config, lib::sr_ecp &_sr_ecp) :
	smb::robot(lib::smb2::ROBOT_NAME, _config, _sr_ecp)
{

}

robot::robot(common::task::task_base& _ecp_object) :
	smb::robot(lib::smb2::ROBOT_NAME, _ecp_object)
{

}

} // namespace smb
} // namespace ecp
} // namespace mrrocpp

