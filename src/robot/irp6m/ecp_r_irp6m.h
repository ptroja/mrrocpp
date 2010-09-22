// -------------------------------------------------------------------------
//                            ecp_local.h dla QNX6
// Definicje struktur danych i metod dla procesow ECP
// robot - irp6_mechatronika
//
// -------------------------------------------------------------------------

#if !defined(_ECP_R_IRP6_MECHATRONIKA_H)
#define _ECP_R_IRP6_MECHATRONIKA_H

#include "base/ecp/ecp_robot.h"
#include "robot/irp6m/const_irp6m.h"

namespace mrrocpp {
namespace ecp {
namespace irp6m {

// ---------------------------------------------------------------
class robot: public common::robot::ecp_robot
{
	// Klasa dla robota irp6_postument (sztywnego)

	public:
	robot (lib::configurator &_config, lib::sr_ecp &_sr_ecp);
	robot (common::task::task& _ecp_object);



}; // end: class ecp_irp6_mechatronika_robot
// ---------------------------------------------------------------

} // namespace irp6m
} // namespace ecp
} // namespace mrrocpp

#endif
