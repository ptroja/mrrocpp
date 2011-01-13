// -------------------------------------------------------------------------
//                            ecp_local.h dla QNX6
// Definicje struktur danych i metod dla procesow ECP
// robot - irp6_mechatronika
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
	robot (common::task::task& _ecp_object);

}; // end: class ecp_irp6_mechatronika_robot
// ---------------------------------------------------------------

} // namespace polycrank
} // namespace ecp
} // namespace mrrocpp

/*
class robot : public common::robot::ecp_robot
{
public:
		//constructor called from UI
		//_config configuration object reference
	 	//_sr_ecp sr_ecp communication object reference
		robot(lib::configurator &_config, lib::sr_ecp &_sr_ecp);

		//constructor called from ECP
		//_ecp_object ecp tak object reference
		robot(common::task::task& _ecp_object);

}; // end: class ecp_conveyor_robot
*/
#endif

