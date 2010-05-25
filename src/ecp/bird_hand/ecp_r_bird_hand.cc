// -------------------------------------------------------------------------
//                            ecp.cc
//            Effector Control Process (lib::ECP) - methods
// Funkcje do tworzenia procesow ECP
// robot - irp6_mechatronika
//
// -------------------------------------------------------------------------

#include "lib/impconst.h"
#include "lib/com_buf.h"
#include "lib/mis_fun.h"

#include "ecp/bird_hand/ecp_r_bird_hand.h"

namespace mrrocpp {
namespace ecp {
namespace bird_hand {

robot::robot(lib::configurator &_config, lib::sr_ecp &_sr_ecp) :
	bird_hand_command_data_port(BIRD_HAND_COMMAND_DATA_PORT),
			bird_hand_configuration_command_data_port(
					BIRD_HAND_CONFIGURATION_DATA_PORT),
			bird_hand_status_reply_data_request_port(
					BIRD_HAND_STATUS_DATA_REQUEST_PORT),
			bird_hand_configuration_reply_data_request_port(
					BIRD_HAND_CONFIGURATION_DATA_REQUEST_PORT), ecp_robot(
					lib::ROBOT_BIRD_HAND, BIRD_HAND_NUM_OF_SERVOS,
					EDP_BIRD_HAND_SECTION, _config, _sr_ecp),
			kinematics_manager() {
	add_data_ports();
	//  Stworzenie listy dostepnych kinematyk.
	create_kinematic_models_for_given_robot();

}

robot::robot(common::task::task& _ecp_object) :
	bird_hand_command_data_port(BIRD_HAND_COMMAND_DATA_PORT),
			bird_hand_configuration_command_data_port(
					BIRD_HAND_CONFIGURATION_DATA_PORT),
			bird_hand_status_reply_data_request_port(
					BIRD_HAND_STATUS_DATA_REQUEST_PORT),
			bird_hand_configuration_reply_data_request_port(
					BIRD_HAND_CONFIGURATION_DATA_REQUEST_PORT), ecp_robot(
					lib::ROBOT_BIRD_HAND, BIRD_HAND_NUM_OF_SERVOS,
					EDP_BIRD_HAND_SECTION, _ecp_object), kinematics_manager() {
	add_data_ports();
	//  Stworzenie listy dostepnych kinematyk.
	create_kinematic_models_for_given_robot();
}

void robot::add_data_ports() {
	port_manager.add_port(&bird_hand_command_data_port);
	port_manager.add_port(&bird_hand_configuration_command_data_port);
	port_manager.add_port(&bird_hand_status_reply_data_request_port);
	port_manager.add_port(&bird_hand_configuration_reply_data_request_port);

}

void robot::clear_data_ports() {
	bird_hand_command_data_port.clear_new_data_flag();
	bird_hand_configuration_command_data_port.clear_new_data_flag();
	bird_hand_status_reply_data_request_port.clear_new_data_flag();
	bird_hand_status_reply_data_request_port.clear_new_request_flag();
	bird_hand_configuration_reply_data_request_port.clear_new_data_flag();
	bird_hand_configuration_reply_data_request_port.clear_new_request_flag();
}

void robot::create_command() {

	bool is_new_data;
	bool is_new_request;

	// NOWE PORTY
	ecp_command.instruction.get_type = NOTHING_DEFINITION;

	if (bird_hand_status_reply_data_request_port.is_new_request()) {
		ecp_command.instruction.get_type |= ARM_DEFINITION;
		is_new_request = true;
	}

	if (bird_hand_configuration_reply_data_request_port.is_new_request()) {
		ecp_command.instruction.get_type |= ROBOT_MODEL_DEFINITION;
		is_new_request = true;
	}

	ecp_command.instruction.set_type = NOTHING_DEFINITION;

	if (bird_hand_command_data_port.get(bird_hand_command_structure) == mrrocpp::lib::NewData) {
		ecp_command.instruction.set_type |= ARM_DEFINITION;

		ecp_edp_cbuffer.bird_hand_command_structure
				= bird_hand_command_structure;
		is_new_data = true;
	}

	if (bird_hand_configuration_command_data_port.get(bird_hand_configuration_command_structure) == mrrocpp::lib::NewData) {
		ecp_command.instruction.set_type |= ROBOT_MODEL_DEFINITION;

		ecp_edp_cbuffer.bird_hand_configuration_command_structure
				= bird_hand_configuration_command_structure;
		is_new_data = true;
	}

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

	// message serialization
	if (communicate_with_edp) {
		memcpy(ecp_command.instruction.arm.serialized_command,
				&ecp_edp_cbuffer, sizeof(ecp_edp_cbuffer));
	}
}

void robot::get_reply() {

	// message deserialization
	memcpy(&edp_ecp_rbuffer, reply_package.arm.serialized_reply,
			sizeof(edp_ecp_rbuffer));

	// generator reply generation
	if (bird_hand_status_reply_data_request_port.is_new_request()) {
		bird_hand_status_reply_structure
				= edp_ecp_rbuffer.bird_hand_status_reply_structure;

		bird_hand_status_reply_data_request_port.set(
				bird_hand_status_reply_structure);
	}

	if (bird_hand_configuration_reply_data_request_port.is_new_request()) {
		bird_hand_configuration_reply_structure
				= edp_ecp_rbuffer.bird_hand_configuration_reply_structure;

		bird_hand_configuration_reply_data_request_port.set(
				bird_hand_configuration_reply_structure);
	}

}

// Stworzenie modeli kinematyki dla robota IRp-6 na postumencie.
void robot::create_kinematic_models_for_given_robot(void) {
	// Stworzenie wszystkich modeli kinematyki.
	add_kinematic_model(new kinematics::bird_hand::kinematic_model_bird_hand());
	// Ustawienie aktywnego modelu.
	set_kinematic_model(0);
}

} // namespace bird_hand
} // namespace ecp
} // namespace mrrocpp

