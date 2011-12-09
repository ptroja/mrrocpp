#if !defined(_ECP_R_SHEAD_H)
#define _ECP_R_SHEAD_H

/*!
 * @file
 * @brief File contains ecp robot class declaration for SwarmItFix Head
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup shead
 */

#include "base/ecp/ecp_robot.h"
#include "dp_shead.h"
#include "base/kinematics/kinematics_manager.h"

namespace mrrocpp {
namespace ecp {
namespace shead {

/*!
 * @brief SwarmItFix Head gripper ecp robot class
 *
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 * @ingroup shead
 */
class robot : public common::robot::ecp_robot, public kinematics::common::kinematics_manager
{
protected:

	/**
	 * @brief EDP command buffer
	 */
	lib::shead::cbuffer ecp_edp_cbuffer;

	/**
	 * @brief EDP reply buffer
	 */
	lib::shead::rbuffer edp_ecp_rbuffer;

	void create_kinematic_models_for_given_robot(void);

public:

	/**
	 * @brief epos motor motion command data port
	 */
	lib::single_thread_port <lib::epos::epos_simple_command> epos_motor_command_data_port;

	/**
	 * @brief epos joint motion command data port
	 */
	lib::single_thread_port <lib::epos::epos_simple_command> epos_joint_command_data_port;

	/**
	 * @brief epos brake command data port
	 */
	lib::single_thread_port <lib::empty_t> epos_brake_command_data_port;

	/**
	 * @brief epos clear fault command data port
	 */
	lib::single_thread_port <lib::empty_t> epos_clear_fault_data_port;

	/**
	 * @brief head soldification command data port
	 */
	lib::single_thread_port <lib::shead::SOLIDIFICATION_ACTIVATION> shead_head_soldification_data_port;

	/**
	 * @brief vacuum activation command data port
	 */
	lib::single_thread_port <lib::shead::VACUUM_ACTIVATION> shead_vacuum_activation_data_port;

	/**
	 * @brief epos motion status reply data request port
	 */
	lib::single_thread_request_port <lib::epos::epos_reply> epos_motor_reply_data_request_port;

	/**
	 * @brief epos motion status with joint reply data request port
	 */
	lib::single_thread_request_port <lib::epos::epos_reply> epos_joint_reply_data_request_port;

	/**
	 * @brief epos motion status with external reply data request port
	 */
	lib::single_thread_request_port <lib::epos::epos_reply> epos_external_reply_data_request_port;

	/**
	 * @brief Head state reply data request port
	 */
	lib::single_thread_request_port <lib::shead::reply> shead_reply_data_request_port;

	/**
	 * @brief constructor called from UI
	 * @param _config configuration object reference
	 * @param _sr_ecp sr_ecp communication object reference
	 */
	robot(const lib::robot_name_t & _robot_name, lib::configurator &_config, lib::sr_ecp &_sr_ecp);

	/**
	 * @brief constructor called from ECP
	 * @param _ecp_object ecp tak object reference
	 */
	robot(const lib::robot_name_t & _robot_name, common::task::task_base& _ecp_object);

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
