// -------------------------------------------------------------------------
//                            ecp.cc
//            Effector Control Process (lib::ECP) - methods
// Funkcje do tworzenia procesow ECP
// robot - irp6_mechatronika
//
// -------------------------------------------------------------------------

#include "lib/impconst.h"
#include "lib/com_buf.h"
#include "lib/mis_fun.h"

#include "ecp/spkm/ecp_r_spkm.h"

namespace mrrocpp {
namespace ecp {
namespace spkm {

robot::robot(lib::configurator &_config, lib::sr_ecp &_sr_ecp) :
	robot_epos_command_data_port(EPOS_COMMAND_DATA_PORT),
			robot_epos_reply_data_port(EPOS_REPLY_DATA_PORT), ecp_robot(
					lib::ROBOT_SPKM, SPKM_NUM_OF_SERVOS, EDP_SPKM_SECTION,
					_config, _sr_ecp) {
}

robot::robot(common::task::task& _ecp_object) :
	robot_epos_command_data_port(EPOS_COMMAND_DATA_PORT),
			robot_epos_reply_data_port(EPOS_REPLY_DATA_PORT), ecp_robot(
					lib::ROBOT_SPKM, SPKM_NUM_OF_SERVOS, EDP_SPKM_SECTION,
					_ecp_object) {
}

void robot::create_ecp_edp_command() {

}

void robot::get_edp_ecp_reply() {

}

} // namespace spkm
} // namespace ecp
} // namespace mrrocpp

