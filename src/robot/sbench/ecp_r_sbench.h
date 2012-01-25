#if !defined(_ECP_R_SBENCH_H)
#define _ECP_R_SBENCH_H

/*!
 * @file
 * @brief File contains ecp robot class declaration for SwarmItFix Bench
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup sbench
 */

#include "base/ecp/ecp_robot.h"
#include "dp_sbench.h"

namespace mrrocpp {
namespace ecp {
namespace sbench {

/*!
 * @brief SwarmItFix Bench robot class.
 *
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 * @ingroup sbench
 */

class robot : public common::robot::_ecp_robot <lib::sbench::c_buffer, lib::sbench::r_buffer>
{
public:

	/**
	 * @brief Bench power supply command data port
	 */
	lib::single_thread_port <lib::sbench::power_supply_state> power_supply_data_port;

	/**
	 * @brief Bench pressure command data port
	 */
	lib::single_thread_port <lib::sbench::cleaning_state> cleaning_state_data_port;

	/**
	 * @brief Bench state reply data request port
	 */
	lib::single_thread_request_port <lib::sbench::rbuffer> data_request_port;

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
	robot(common::task::task_base& _ecp_object);

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
