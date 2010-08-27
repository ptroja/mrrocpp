#if !defined(_ECP_R_CONVEYOR_H)
#define _ECP_R_CONVEYOR_H

/*!
 * @file ecp_r_conv.h
 * @brief File contains ecp robot class declaration for Conveyor
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup conveyor
 */

#include "base/ecp/ecp_robot.h"
#include "robot/conveyor/conveyor_const.h"

namespace mrrocpp {
namespace ecp {
namespace conveyor {

// ---------------------------------------------------------------
class robot : public common::ecp_robot
{
	// Klasa dla robota conveyor

public:
	robot(lib::configurator &_config, lib::sr_ecp &_sr_ecp);
	robot(common::task::task& _ecp_object);

}; // end: class ecp_conveyor_robot
// ---------------------------------------------------------------

} // namespace conveyor
} // namespace ecp
} // namespace mrrocpp

#endif
