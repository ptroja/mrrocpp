#if !defined(_ECP_R_BIRD_HAND_H)
#define _ECP_R_BIRD_HAND_H

/*!
 * @file
 * @brief File contains ecp robot class declaration for Bird Hand three finger gripper
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup bird_hand
 */

#include "base/ecp/ecp_robot.h"
#include "robot/bird_hand/const_bird_hand.h"
#include "base/kinematics/kinematics_manager.h"
#include "robot/bird_hand/kinematic_model_bird_hand.h"

namespace mrrocpp {
namespace ecp {
namespace bird_hand {

/*!
 * @brief Bird Hand gripper ecp robot class
 *
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 * @ingroup bird_hand
 */
class robot : public common::robot::ecp_robot, public kinematics::common::kinematics_manager
{
protected:

	/**
	 * @brief motion command data port
	 */
	lib::single_thread_port <lib::bird_hand::command> bird_hand_command_data_port;

	/**
	 * @brief configuration command data port
	 */
	lib::single_thread_port <lib::bird_hand::configuration> bird_hand_configuration_command_data_port;

	/**
	 * @brief Joints state reply data request port
	 */
	lib::single_thread_request_port <lib::bird_hand::status> bird_hand_status_reply_data_request_port;

	/**
	 * @brief configuration reply data request port
	 */
	lib::single_thread_request_port <lib::bird_hand::configuration> bird_hand_configuration_reply_data_request_port;

	/**
	 * @brief EDP command buffer
	 */
	lib::bird_hand::cbuffer ecp_edp_cbuffer;

	/**
	 * @brief EDP reply buffer
	 */
	lib::bird_hand::rbuffer edp_ecp_rbuffer;

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

} // namespace bird_hand
} // namespace ecp
} // namespace mrrocpp

#endif
