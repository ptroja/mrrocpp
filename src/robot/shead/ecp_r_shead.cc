/*!
 * @file
 * @brief File contains ecp robot class definition for SwarmItFix Head
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup shead
 */

#include "base/lib/impconst.h"

#include "robot/shead/ecp_r_shead.h"

namespace mrrocpp {
namespace ecp {
namespace shead {

robot::robot(lib::configurator &_config, lib::sr_ecp &_sr_ecp) :
	robot::ecp_robot(lib::shead::ROBOT_NAME, lib::shead::NUM_OF_SERVOS, lib::shead::EDP_SECTION, _config, _sr_ecp),
			kinematics_manager(),
			shead_head_soldification_data_port(lib::shead::HEAD_SOLIDIFICATION_DATA_PORT, port_manager),
			shead_vacuum_activation_data_port(lib::shead::VACUUM_ACTIVATION_DATA_PORT, port_manager),
			shead_reply_data_request_port(lib::shead::REPLY_DATA_REQUEST_PORT, port_manager)

{
	//  Stworzenie listy dostepnych kinematyk.
	create_kinematic_models_for_given_robot();
}

robot::robot(common::task::task& _ecp_object) :
	robot::ecp_robot(lib::shead::ROBOT_NAME, lib::shead::NUM_OF_SERVOS, lib::shead::EDP_SECTION, _ecp_object),
			kinematics_manager(),
			shead_head_soldification_data_port(lib::shead::HEAD_SOLIDIFICATION_DATA_PORT, port_manager),
			shead_vacuum_activation_data_port(lib::shead::VACUUM_ACTIVATION_DATA_PORT, port_manager),
			shead_reply_data_request_port(lib::shead::REPLY_DATA_REQUEST_PORT, port_manager)

{
	//  Stworzenie listy dostepnych kinematyk.
	create_kinematic_models_for_given_robot();
}

// Stworzenie modeli kinematyki dla robota IRp-6 na postumencie.
void robot::create_kinematic_models_for_given_robot(void)
{
	// Stworzenie wszystkich modeli kinematyki.
	add_kinematic_model(new kinematics::shead::model());
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

	if (shead_head_soldification_data_port.get() == mrrocpp::lib::NewData) {
		ecp_command.instruction.set_type = ARM_DEFINITION;

		// generator command interpretation
		// narazie proste przepisanie

		ecp_edp_cbuffer.variant = lib::shead::CBUFFER_HEAD_SOLIDIFICATION;

		ecp_edp_cbuffer.head_solidification = shead_head_soldification_data_port.data;

		if (is_new_data) {
			throw common::robot::ECP_error(lib::NON_FATAL_ERROR, INVALID_COMMAND_TO_EDP);
		} else {
			is_new_data = true;
		}
	}

	if (shead_vacuum_activation_data_port.get() == mrrocpp::lib::NewData) {
		ecp_command.instruction.set_type = ARM_DEFINITION;

		// generator command interpretation
		// narazie proste przepisanie

		ecp_edp_cbuffer.variant = lib::shead::CBUFFER_VACUUM_ACTIVATION;

		ecp_edp_cbuffer.vacuum_activation = shead_vacuum_activation_data_port.data;

		if (is_new_data) {
			throw common::robot::ECP_error(lib::NON_FATAL_ERROR, INVALID_COMMAND_TO_EDP);
		} else {
			is_new_data = true;
		}

	}

	is_new_request = shead_reply_data_request_port.is_new_request();

	communicate_with_edp = true;

	if (is_new_data && is_new_request) {
		ecp_command.instruction.instruction_type = lib::SET_GET;
	} else if (is_new_data) {
		ecp_command.instruction.instruction_type = lib::SET;
	} else if (is_new_request) {
		ecp_command.instruction.instruction_type = lib::GET;
	} else {
		communicate_with_edp = false;
	}

	if (is_new_request) {
		ecp_command.instruction.get_type = ARM_DEFINITION;
	}

	// message serialization
	if (communicate_with_edp) {
		memcpy(ecp_command.instruction.arm.serialized_command, &ecp_edp_cbuffer, sizeof(ecp_edp_cbuffer));
	}
}

void robot::get_reply()
{

	// message deserialization
	memcpy(&edp_ecp_rbuffer, reply_package.arm.serialized_reply, sizeof(edp_ecp_rbuffer));

	// generator reply generation

	if (shead_reply_data_request_port.is_new_request()) {
		shead_reply_data_request_port.data = edp_ecp_rbuffer.shead_reply;

		shead_reply_data_request_port.set();
	}

}

} // namespace smb
} // namespace ecp
} // namespace mrrocpp

