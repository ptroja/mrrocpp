/*!
 * @file
 * @brief File contains ecp robot class definition for SwarmItFix head
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup shead
 */

#include "ecp_r_shead1.h"

namespace mrrocpp {
namespace ecp {
namespace shead1 {

robot::robot(lib::configurator &_config, lib::sr_ecp &_sr_ecp) :
	shead::robot(lib::shead1::ROBOT_NAME, _config, _sr_ecp)
{

}

robot::robot(common::task::task_base& _ecp_object) :
	shead::robot(lib::shead1::ROBOT_NAME, _ecp_object)
{

}

} // namespace shead
} // namespace ecp
} // namespace mrrocpp

