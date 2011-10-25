// -------------------------------------------------------------------------
//                            ecp_local.h dla QNX6.5
// Definicje struktur danych i metod dla procesow ECP
// robot - polycrank
//
// -------------------------------------------------------------------------

#if !defined(_ECP_R_POLYCRANK_H)
#define _ECP_R_POLYCRANK_H

#include "base/ecp/ecp_robot.h"
#include "robot/polycrank/const_polycrank.h"
//#include "base/ecp/ecp_robot.h"
//#include "robot/conveyor/const_conveyor.h"

namespace mrrocpp {
namespace ecp {
namespace polycrank {

// ---------------------------------------------------------------
class robot: public common::robot::ecp_robot
{
	// Klasa dla robota irp6_postument (sztywnego)

	public:
	robot (lib::configurator &_config, lib::sr_ecp &_sr_ecp);
	robot (common::task::task_base& _ecp_object);

}; // end: class ecp_irp6_mechatronika_robot
// ---------------------------------------------------------------

} // namespace polycrank
} // namespace ecp
} // namespace mrrocpp

#endif

