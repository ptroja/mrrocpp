#if !defined(_ECP_R_SPKM_H)
#define _ECP_R_SPKM_H

/*!
 * @file
 * @brief File contains ecp robot class declaration for SwarmItFix Parallel Kinematic Machine
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup spkm
 */

#include "base/ecp/ecp_robot.h"
#include "robot/spkm/const_spkm.h"
#include "base/kinematics/kinematics_manager.h"
#include "robot/spkm/kinematic_model_spkm.h"

namespace mrrocpp {
namespace ecp {
namespace spkm {

/*!
 * @brief SwarmItFix Parallel Kinematic Machine gripper ecp robot class
 *
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 * @ingroup spkm
 */
class robot : public common::robot::ecp_robot, public kinematics::common::kinematics_manager
{
protected:
	//bufory wejsciowe z generatora

	/**
	 * @brief epos cubic motion command data port
	 */
	lib::single_thread_port <lib::epos::epos_cubic_command> epos_cubic_command_data_port;

	/**
	 * @brief epos trapezoidal motion command data port
	 */
	lib::single_thread_port <lib::epos::epos_trapezoidal_command> epos_trapezoidal_command_data_port;

	/**
	 * @brief epos operational motion command data port
	 */
	lib::single_thread_port <lib::epos::epos_operational_command> epos_operational_command_data_port;

	/**
	 * @brief epos brake command data port
	 */
	lib::single_thread_port <bool> epos_brake_command_data_port;

	/**
	 * @brief epos motion status reply data request port
	 */
	lib::single_thread_request_port <lib::epos::epos_reply> epos_reply_data_request_port;

	/**
	 * @brief EDP command buffer
	 */
	lib::spkm::cbuffer ecp_edp_cbuffer;

	/**
	 * @brief EDP reply buffer
	 */
	lib::spkm::rbuffer edp_ecp_rbuffer;

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
} // namespace spkm
} // namespace ecp
} // namespace mrrocpp

#endif
