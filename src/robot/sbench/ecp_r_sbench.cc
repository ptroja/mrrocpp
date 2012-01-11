/*!
 * @file
 * @brief File contains ecp robot class definition for SwarmItFix Head
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup sbench
 */

#include "base/lib/impconst.h"
#include "base/lib/sr/sr_ecp.h"

#include "ecp_r_sbench.h"
#include "kinematic_model_sbench.h"

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
	//  Stworzenie listy dostepnych kinematyk.
	create_kinematic_models_for_given_robot();
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
	//  Stworzenie listy dostepnych kinematyk.
	create_kinematic_models_for_given_robot();
}

// Stworzenie modeli kinematyki dla robota IRp-6 na postumencie.
void robot::create_kinematic_models_for_given_robot(void)
{
	// Stworzenie wszystkich modeli kinematyki.
	add_kinematic_model(new kinematics::sbench::model());
	// Ustawienie aktywnego modelu.
	set_kinematic_model(0);
}

void robot::create_command()
{
	// checks if any data_port is set
	bool is_new_data = false;

	// cheks if any data_request_port is set
	bool is_new_request = false;

	sr_ecp_msg.message("create_command");

	is_new_data = false;

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

	communicate_with_edp = true;

	if (is_new_data && is_new_request) {
		ecp_command.instruction_type = lib::SET_GET;
	} else if (is_new_data) {
		ecp_command.instruction_type = lib::SET;
	} else if (is_new_request) {
		ecp_command.instruction_type = lib::GET;
	} else {
		communicate_with_edp = false;
	}

	if (is_new_request) {
		ecp_command.get_type = ARM_DEFINITION;
	}

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

