// -------------------------------------------------------------------------
//                            ecp_local.h dla QNX6
// Definicje struktur danych i metod dla procesow ECP
// robot - irp6_mechatronika
//
// -------------------------------------------------------------------------

#if !defined(_ECP_R_SMB_H)
#define _ECP_R_SMB_H

#include "ecp/common/ecp_robot.h"
#include "kinematics/common/kinematics_manager.h"
#include "kinematics/smb/kinematic_model_smb.h"
#include "lib/robot_consts/smb_const.h"
#include "lib/data_port_headers/epos.h"
#include "lib/data_port_headers/smb.h"

namespace mrrocpp {
namespace ecp {
namespace smb {

// ---------------------------------------------------------------
class robot: public common::ecp_robot,
		public kinematics::common::kinematics_manager {
	// Klasa dla robota irp6_postument (sztywnego)
protected:
	//bufory wejsciowe z generatora
	//epos.h
	lib::single_thread_port<lib::epos_low_level_command>
			epos_low_level_command_data_port;
	lib::epos_low_level_command epos_data_port_command_structure;

	lib::single_thread_port<lib::epos_gen_parameters>
			epos_gen_parameters_data_port;
	lib::epos_gen_parameters epos_data_port_gen_parameters_structure;
	//smb.h
	lib::single_thread_port<lib::smb_multi_pin_insertion>
			smb_multi_pin_insertion_data_port;
	lib::smb_multi_pin_insertion smb_multi_pin_insertion_structure;

	lib::single_thread_port<lib::smb_multi_pin_locking>
			smb_multi_pin_locking_data_port;
	lib::smb_multi_pin_locking smb_multi_pin_locking_structure;

	// bufory wyjsciowe do generatora
	// epos.h
	lib::single_thread_request_port<lib::epos_reply>
			epos_reply_data_request_port;
	lib::epos_reply epos_data_port_reply_structure;

	//smb.h
	lib::single_thread_request_port<lib::smb_multi_leg_reply>
			smb_multi_leg_reply_data_request_port;
	lib::smb_multi_leg_reply smb_multi_leg_reply_data_request_port_structure;

	// bufory do edp
	lib::smb_cbuffer ecp_edp_cbuffer;
	lib::smb_rbuffer edp_ecp_rbuffer;

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

} // namespace smb
} // namespace ecp
} // namespace mrrocpp

#endif
