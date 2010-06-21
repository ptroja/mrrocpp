// -------------------------------------------------------------------------
//                            ecp.h dla QNX6
// Definicje struktur danych i metod dla procesow ECP
// robot - conveyor
//
// -------------------------------------------------------------------------

#if !defined(_ECP_R_CONVEYOR_H)
#define _ECP_R_CONVEYOR_H

#include "ecp/common/ecp_robot.h"
#include "robot/conveyor/conveyor_const.h"

namespace mrrocpp {
namespace ecp {
namespace conveyor {

// ---------------------------------------------------------------
class robot: public common::ecp_robot
{
	// Klasa dla robota conveyor

	public:
	robot (lib::configurator &_config, lib::sr_ecp &_sr_ecp);
	robot (common::task::task& _ecp_object);


}; // end: class ecp_conveyor_robot
// ---------------------------------------------------------------

} // namespace conveyor
} // namespace ecp
} // namespace mrrocpp

#endif
