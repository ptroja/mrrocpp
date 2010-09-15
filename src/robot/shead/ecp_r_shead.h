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
#include "robot/shead/const_shead.h"
#include "base/kinematics/kinematics_manager.h"
#include "robot/shead/kinematic_model_shead.h"

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
	 * @brief head soldification command data port
	 */
	lib::single_thread_port <lib::shead::HEAD_SOLIDIFICATION> shead_head_soldification_data_port;

	/**
	 * @brief vacuum activation command data port
	 */
	lib::single_thread_port <lib::shead::VACUUM_ACTIVATION> shead_vacuum_activation_data_port;

	/**
	 * @brief Head state reply data request port
	 */
	lib::single_thread_request_port <lib::shead::reply> shead_reply_data_request_port;

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
