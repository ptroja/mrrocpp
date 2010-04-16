// -------------------------------------------------------------------------
//                            ecp_local.h dla QNX6
// Definicje struktur danych i metod dla procesow ECP
// robot - irp6_mechatronika
//
// -------------------------------------------------------------------------

#if !defined(_ECP_R_SPKM_H)
#define _ECP_R_SPKM_H

#include "ecp/common/ecp_robot.h"
#include "lib/robot_consts/spkm_const.h"
#include "lib/data_port_headers/epos.h"

namespace mrrocpp {
namespace ecp {
namespace spkm {

// ---------------------------------------------------------------
class robot: public common::ecp_robot {
	// Klasa dla robota irp6_postument (sztywnego)
protected:
	lib::single_thread_port<epos_command> robot_epos_command_data_port;
	lib::single_thread_port<epos_reply> robot_epos_reply_data_port;
	epos_command robot_epos_command;
	epos_reply robot_epos_reply;

	void add_data_ports();

public:
	robot(lib::configurator &_config, lib::sr_ecp &_sr_ecp);
	robot(common::task::task& _ecp_object);


}; // end: class ecp_irp6_mechatronika_robot
// ---------------------------------------------------------------

} // namespace spkm
} // namespace ecp
} // namespace mrrocpp

#endif
