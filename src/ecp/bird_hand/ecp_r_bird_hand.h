// -------------------------------------------------------------------------
//                            ecp_local.h dla QNX6
// Definicje struktur danych i metod dla procesow ECP
// robot - irp6_mechatronika
//
// -------------------------------------------------------------------------

#if !defined(_ECP_R_BIRD_HAND_H)
#define _ECP_R_BIRD_HAND_H

#include "ecp/common/ecp_robot.h"
#include "lib/robot_consts/bird_hand_const.h"
#include "kinematics/common/kinematics_manager.h"
#include "kinematics/bird_hand/kinematic_model_bird_hand.h"
#include "lib/data_port_headers/bird_hand.h"

namespace mrrocpp {
namespace ecp {
namespace bird_hand {

// ---------------------------------------------------------------
class robot: public common::ecp_robot,
		public kinematics::common::kinematics_manager {

protected:

	// zadawanie rozkazu
	lib::single_thread_port<lib::bird_hand_command> bird_hand_command_data_port;
	lib::bird_hand_command bird_hand_command_structure;

	// zadawanie parametrow konfiguracji
	lib::single_thread_port<lib::bird_hand_configuration>
			bird_hand_configuration_command_data_port;
	lib::bird_hand_configuration bird_hand_configuration_command_structure;

	// odbieranie statusu robota
	lib::single_thread_request_port<lib::bird_hand_status>
			bird_hand_status_reply_data_request_port;
	lib::bird_hand_status bird_hand_status_reply_structure;

	// odczytanie parametrow konfiguracji
	lib::single_thread_request_port<lib::bird_hand_configuration>
			bird_hand_configuration_reply_data_request_port;
	lib::bird_hand_configuration bird_hand_configuration_reply_structure;

	// bufory do edp
	lib::bird_hand_cbuffer ecp_edp_cbuffer;
	lib::bird_hand_rbuffer edp_ecp_rbuffer;

	void create_kinematic_models_for_given_robot(void);
	void add_data_ports();

public:
	robot(lib::configurator &_config, lib::sr_ecp &_sr_ecp);
	robot(common::task::task& _ecp_object);

	void create_command();
	void get_reply();
	void clear_data_ports();

}; // end: class ecp_irp6_mechatronika_robot
// ---------------------------------------------------------------

} // namespace bird_hand
} // namespace ecp
} // namespace mrrocpp

#endif
