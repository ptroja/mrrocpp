/*!
 * @file
 * @brief File contains ecp robot class definition for SwarmItFix Parallel Kinematic Machine
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup spkm
 */

#include "ecp_r_spkm2.h"

namespace mrrocpp {
namespace ecp {
namespace spkm2 {

robot::robot(lib::configurator &_config, lib::sr_ecp &_sr_ecp) :
	spkm::robot(lib::spkm2::ROBOT_NAME, _config, _sr_ecp)
{
}

robot::robot(common::task::task_base& _ecp_object) :
	spkm::robot(lib::spkm2::ROBOT_NAME, _ecp_object)
{
}

} // namespace spkm
} // namespace ecp
} // namespace mrrocpp
