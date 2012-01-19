/*!
 * @file
 * @brief File contains ecp robot class definition for SwarmItFix Bench
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup sbench
 */

#include "base/lib/impconst.h"
#include "base/lib/sr/sr_ecp.h"

#include "ecp_r_sbench.h"

namespace mrrocpp {
namespace ecp {
namespace sbench {

robot::robot(lib::configurator &_config, lib::sr_ecp &_sr_ecp) :
		ecp::common::robot::_ecp_robot <lib::sbench::c_buffer, lib::sbench::r_buffer>(lib::sbench::ROBOT_NAME, lib::sbench::NUM_OF_SERVOS, _config, _sr_ecp)
		,
		sbench_command_voltage_data_port(lib::sbench::COMMAND_DATA_VOLTAGE_PORT, port_manager)
		,
		sbench_command_preasure_data_port(lib::sbench::COMMAND_DATA_PREASURE_PORT, port_manager)
		,
		sbench_reply_data_request_port(lib::sbench::REPLY_DATA_REQUEST_PORT, port_manager)
{
	data_ports_used = true;
}

robot::robot(common::task::task_base& _ecp_object) :
		ecp::common::robot::_ecp_robot <lib::sbench::c_buffer, lib::sbench::r_buffer>(lib::sbench::ROBOT_NAME, lib::sbench::NUM_OF_SERVOS, _ecp_object)
		,
		sbench_command_voltage_data_port(lib::sbench::COMMAND_DATA_VOLTAGE_PORT, port_manager)
		,
		sbench_command_preasure_data_port(lib::sbench::COMMAND_DATA_PREASURE_PORT, port_manager)
		,
		sbench_reply_data_request_port(lib::sbench::REPLY_DATA_REQUEST_PORT, port_manager)
{
	data_ports_used = true;
}


void robot::create_command()
{

	sr_ecp_msg.message("create_command");

	if (sbench_command_voltage_data_port.get() == mrrocpp::lib::single_thread_port_interface::NewData) {
		ecp_command.set_type = ARM_DEFINITION;

		ecp_command.sbench.variant = lib::sbench::VOLTAGE;
		// generator command interpretation

		ecp_command.sbench.voltage_buf = sbench_command_voltage_data_port.data;

		check_then_set_command_flag(is_new_data);
	}

	if (sbench_command_preasure_data_port.get() == mrrocpp::lib::single_thread_port_interface::NewData) {
		ecp_command.set_type = ARM_DEFINITION;

		ecp_command.sbench.variant = lib::sbench::PREASURE;
		// generator command interpretation

		ecp_command.sbench.preasure_buf = sbench_command_preasure_data_port.data;

		check_then_set_command_flag(is_new_data);
	}

	is_new_request = sbench_reply_data_request_port.is_new_request();

}

void robot::get_reply()
{
	if (sbench_reply_data_request_port.is_new_request()) {
		sbench_reply_data_request_port.data = reply_package.sbench;
		sbench_reply_data_request_port.set();
	}
}

} // namespace smb
} // namespace ecp
} // namespace mrrocpp

