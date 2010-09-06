#if !defined(_ECP_R_SMB_H)
#define _ECP_R_SMB_H

/*!
 * @file
 * @brief File contains ecp robot class declaration for SwarmItFix Mobile Base
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup smb
 */

#include "base/ecp/ecp_robot.h"
#include "base/kinematics/kinematics_manager.h"
#include "robot/smb/kinematic_model_smb.h"
#include "robot/smb/const_smb.h"

namespace mrrocpp {
namespace ecp {
namespace smb {

// ---------------------------------------------------------------
class robot : public common::robot::ecp_robot, public kinematics::common::kinematics_manager
{

	// Klasa dla robota irp6_postument (sztywnego)
protected:
	void create_kinematic_models_for_given_robot(void);
	//bufory wejsciowe z generatora
	//epos.h
	lib::single_thread_port <lib::epos::epos_cubic_command> epos_cubic_command_data_port;

	lib::single_thread_port <lib::epos::epos_trapezoidal_command> epos_trapezoidal_command_data_port;

	/*
	 lib::single_thread_port<lib::epos_gen_parameters>
	 epos_gen_parameters_data_port;
	 lib::epos_gen_parameters epos_gen_parameters_structure;
	 */
	//smb.h
	lib::single_thread_port <lib::smb::multi_pin_insertion_td> smb_multi_pin_insertion_data_port;

	lib::single_thread_port <lib::smb::multi_pin_locking_td> smb_multi_pin_locking_data_port;

	// bufory wyjsciowe do generatora
	// epos.h
	lib::single_thread_request_port <lib::epos::epos_reply> epos_reply_data_request_port;

	//smb.h
	lib::single_thread_request_port <lib::smb::multi_leg_reply_td> smb_multi_leg_reply_data_request_port;

	// bufory do edp
	lib::smb::cbuffer ecp_edp_cbuffer;
	lib::smb::rbuffer edp_ecp_rbuffer;

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
