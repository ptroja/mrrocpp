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

/*!
 * @brief SwarmItFix Mobile Base gripper ecp robot class
 *
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 * @ingroup smb
 */
class robot : public common::robot::ecp_robot, public kinematics::common::kinematics_manager
{

protected:
	void create_kinematic_models_for_given_robot(void);

	/**
	 * @brief epos cubic motion command data port
	 */
	lib::single_thread_port <lib::epos::epos_cubic_command> epos_cubic_command_data_port;

	/**
	 * @brief epos trapezoidal motion command data port
	 */
	lib::single_thread_port <lib::epos::epos_trapezoidal_command> epos_trapezoidal_command_data_port;

	/**
	 * @brief pin insertion command data port
	 */
	lib::single_thread_port <lib::smb::multi_pin_insertion_td> smb_multi_pin_insertion_data_port;

	/**
	 * @brief pin locking command data port
	 */
	lib::single_thread_port <lib::smb::multi_pin_locking_td> smb_multi_pin_locking_data_port;

	/**
	 * @brief epos motion status reply data request port
	 */
	lib::single_thread_request_port <lib::epos::epos_reply> epos_reply_data_request_port;

	/**
	 * @brief leg status reply data request port
	 */
	lib::single_thread_request_port <lib::smb::multi_leg_reply_td> smb_multi_leg_reply_data_request_port;

	/**
	 * @brief EDP command buffer
	 */
	lib::smb::cbuffer ecp_edp_cbuffer;

	/**
	 * @brief EDP reply buffer
	 */
	lib::smb::rbuffer edp_ecp_rbuffer;

public:
	/**
	 * @brief constructor called from UI
	 * @param _config configuration object reference
	 * @param _sr_ecp sr_ecp communication object reference
	 */
	robot(lib::configurator &_config, lib::sr_ecp &_sr_ecp);

	/**
	 * @brief constructor called from ECP
	 * @param _ecp_object ecp tak object reference
	 */
	robot(common::task::task& _ecp_object);

	/**
	 * @brief set the edp command buffer
	 * basing on data_ports
	 */
	void create_command();

	/**
	 * @brief set the data_request_ports
	 * basing on edp reply buffer
	 */
	void get_reply();

};

} // namespace smb
} // namespace ecp
} // namespace mrrocpp

#endif
