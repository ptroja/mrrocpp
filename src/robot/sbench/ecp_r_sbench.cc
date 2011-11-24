/*!
 * @file
 * @brief File contains ecp robot class definition for SwarmItFix Head
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup sbench
 */

#include "base/lib/impconst.h"
#include "base/lib/sr/sr_ecp.h"

#include "robot/sbench/ecp_r_sbench.h"
#include "robot/sbench/kinematic_model_sbench.h"

namespace mrrocpp {
namespace ecp {
namespace sbench {

robot::robot(lib::configurator &_config, lib::sr_ecp &_sr_ecp) :
		ecp::common::robot::ecp_robot(lib::sbench::ROBOT_NAME, lib::sbench::NUM_OF_SERVOS, _config, _sr_ecp), sbench_command_data_port(lib::sbench::COMMAND_DATA_PORT, port_manager), sbench_reply_data_request_port(lib::sbench::REPLY_DATA_REQUEST_PORT, port_manager)
{
	//  Stworzenie listy dostepnych kinematyk.
	create_kinematic_models_for_given_robot();
}

robot::robot(common::task::task_base& _ecp_object) :
		ecp::common::robot::ecp_robot(lib::sbench::ROBOT_NAME, lib::sbench::NUM_OF_SERVOS, _ecp_object), sbench_command_data_port(lib::sbench::COMMAND_DATA_PORT, port_manager), sbench_reply_data_request_port(lib::sbench::REPLY_DATA_REQUEST_PORT, port_manager)
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
	//	int new_data_counter;
	bool is_new_data;
	bool is_new_request;

	sr_ecp_msg.message("create_command");

	is_new_data = false;

	if (sbench_command_data_port.get() == mrrocpp::lib::NewData) {
		ecp_command.set_type = ARM_DEFINITION;
		// generator command interpretation
		for (int i = 0; i < lib::sbench::NUM_OF_PINS; ++i) {

			ecp_edp_cbuffer.pins_state[i] = sbench_command_data_port.data[i];
		}

		if (is_new_data) {
			BOOST_THROW_EXCEPTION(exception::nfe_r() << lib::exception::mrrocpp_error0(INVALID_COMMAND_TO_EDP));
		} else {
			is_new_data = true;
		}
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

	// message serialization
	if (communicate_with_edp) {
		memcpy(ecp_command.serialized_command, &ecp_edp_cbuffer, sizeof(ecp_edp_cbuffer));
		assert(sizeof(ecp_command.serialized_command) >= sizeof(ecp_edp_cbuffer));
	}
}

void robot::get_reply()
{

	// message deserialization
	memcpy(&edp_ecp_rbuffer, reply_package.serialized_reply, sizeof(edp_ecp_rbuffer));

	// generator reply generation

	if (sbench_reply_data_request_port.is_new_request()) {
		for (int i = 0; i < lib::sbench::NUM_OF_PINS; ++i) {
			sbench_reply_data_request_port.data[i] = edp_ecp_rbuffer.pins_state[i];
		}

		sbench_reply_data_request_port.set();
	}

}

} // namespace smb
} // namespace ecp
} // namespace mrrocpp

