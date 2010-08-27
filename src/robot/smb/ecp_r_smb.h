// -------------------------------------------------------------------------
//                            ecp_local.h dla QNX6
// Definicje struktur danych i metod dla procesow ECP
// robot - irp6_mechatronika
//
// -------------------------------------------------------------------------

#if !defined(_ECP_R_SMB_H)
#define _ECP_R_SMB_H

#include "base/ecp/ecp_robot.h"
#include "base/kinematics/kinematics_manager.h"
#include "robot/smb/kinematic_model_smb.h"
#include "robot/smb/const_smb.h"

namespace mrrocpp {
namespace ecp {
namespace smb {

// ---------------------------------------------------------------
class robot : public common::ecp_robot, public kinematics::common::kinematics_manager
{

	// Klasa dla robota irp6_postument (sztywnego)
protected:
	void create_kinematic_models_for_given_robot(void);
	//bufory wejsciowe z generatora
	//epos.h
	lib::single_thread_port <lib::epos_cubic_command> epos_cubic_command_data_port;
	lib::epos_cubic_command epos_cubic_command_structure;

	lib::single_thread_port <lib::epos_trapezoidal_command> epos_trapezoidal_command_data_port;
	lib::epos_trapezoidal_command epos_trapezoidal_command_structure;

	/*
	 lib::single_thread_port<lib::epos_gen_parameters>
	 epos_gen_parameters_data_port;
	 lib::epos_gen_parameters epos_gen_parameters_structure;
	 */
	//smb.h
	lib::single_thread_port <lib::smb_multi_pin_insertion> smb_multi_pin_insertion_data_port;
	lib::smb_multi_pin_insertion smb_multi_pin_insertion_structure;

	lib::single_thread_port <lib::smb_multi_pin_locking> smb_multi_pin_locking_data_port;
	lib::smb_multi_pin_locking smb_multi_pin_locking_structure;

	// bufory wyjsciowe do generatora
	// epos.h
	lib::single_thread_request_port <lib::epos_reply> epos_reply_data_request_port;
	lib::epos_reply epos_reply_structure;

	//smb.h
	lib::single_thread_request_port <lib::smb_multi_leg_reply> smb_multi_leg_reply_data_request_port;

	lib::smb_multi_leg_reply smb_multi_leg_reply_structure;

	// bufory do edp
	lib::smb_cbuffer ecp_edp_cbuffer;
	lib::smb_rbuffer edp_ecp_rbuffer;

public:
	robot(lib::configurator &_config, lib::sr_ecp &_sr_ecp);
	robot(common::task::task& _ecp_object);

	void create_command();
	void get_reply();

}; // end: class ecp_irp6_mechatronika_robot
// ---------------------------------------------------------------


} // namespace smb
} // namespace ecp
} // namespace mrrocpp

#endif
