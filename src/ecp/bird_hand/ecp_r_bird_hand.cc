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
			bird_hand_low_level_command_data_port(
					BIRD_HAND_LOW_LEVEL_COMMAND_DATA_PORT),
			bird_hand_gen_parameters_data_port(
					BIRD_HAND_GEN_PARAMETERS_DATA_PORT),
			bird_hand_reply_data_request_port(BIRD_HAND_REPLY_DATA_REQUEST_PORT),
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
			bird_hand_low_level_command_data_port(
					BIRD_HAND_LOW_LEVEL_COMMAND_DATA_PORT),
			bird_hand_gen_parameters_data_port(
					BIRD_HAND_GEN_PARAMETERS_DATA_PORT),
			bird_hand_reply_data_request_port(BIRD_HAND_REPLY_DATA_REQUEST_PORT),
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
	// STARE PORTY
	port_manager.add_port(&bird_hand_low_level_command_data_port);
	port_manager.add_port(&bird_hand_gen_parameters_data_port);
	port_manager.add_port(&bird_hand_reply_data_request_port);

	// NOWE PORTY
	port_manager.add_port(&bird_hand_command_data_port);
	port_manager.add_port(&bird_hand_configuration_command_data_port);
	port_manager.add_port(&bird_hand_status_reply_data_request_port);
	port_manager.add_port(&bird_hand_configuration_reply_data_request_port);

}

void robot::clear_data_ports() {
	// STARE PORTY
	bird_hand_low_level_command_data_port.clear_new_data_flag();
	bird_hand_gen_parameters_data_port.clear_new_data_flag();
	bird_hand_reply_data_request_port.clear_new_request_flag();
	bird_hand_reply_data_request_port.clear_new_data_flag();

	// NOWE PORTY
	bird_hand_command_data_port.clear_new_data_flag();
	bird_hand_configuration_command_data_port.clear_new_data_flag();
	bird_hand_status_reply_data_request_port.clear_new_data_flag();
	bird_hand_status_reply_data_request_port.clear_new_request_flag();
	bird_hand_configuration_reply_data_request_port.clear_new_data_flag();
	bird_hand_configuration_reply_data_request_port.clear_new_request_flag();
}

void robot::create_command() {

	int new_data_counter;
	bool is_new_data;
	bool is_new_request;

	sr_ecp_msg.message("create_command");

	new_data_counter = 0;

	if (bird_hand_low_level_command_data_port.is_new_data()) {
		new_data_counter++;
	}

	if (bird_hand_low_level_command_data_port.is_new_data()) {
		new_data_counter++;
	}

	if (new_data_counter == 0) {
		is_new_data = false;
	} else if (new_data_counter == 1) {
		is_new_data = true;
	} else if (new_data_counter > 1) {
		is_new_data = false;
		throw ecp_robot::ECP_error(lib::NON_FATAL_ERROR, INVALID_COMMAND_TO_EDP);
	}

	is_new_request = bird_hand_reply_data_request_port.is_new_request();

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

	if (bird_hand_reply_data_request_port.is_new_request()) {
		ecp_command.instruction.get_type = ARM_DEFINITION; // arm - ORYGINAL
	}

	if (bird_hand_low_level_command_data_port.is_new_data()) {
		ecp_command.instruction.set_type = ARM_DEFINITION;
		bird_hand_low_level_command_structure
				= bird_hand_low_level_command_data_port.get();
		// generator command interpretation
		// narazie proste przepisanie

		ecp_edp_cbuffer.variant
				= lib::BIRD_HAND_CBUFFER_BIRD_HAND_LOW_LEVEL_COMMAND;

		ecp_edp_cbuffer.bird_hand_low_level_command_structure
				= bird_hand_low_level_command_structure;

	} else if (bird_hand_gen_parameters_data_port.is_new_data()) {
		ecp_command.instruction.set_type = ARM_DEFINITION;
		bird_hand_gen_parameters_structure
				= bird_hand_gen_parameters_data_port.get();
		// generator command interpretation
		// narazie proste przepisanie

		ecp_edp_cbuffer.variant
				= lib::BIRD_HAND_CBUFFER_BIRD_HAND_GEN_PARAMETERS;

		ecp_edp_cbuffer.bird_hand_gen_parameters_structure
				= bird_hand_gen_parameters_structure;
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
	for (int i = 0; i < 6; i++) {
		bird_hand_reply_structure.bird_hand_controller[i].position
				= edp_ecp_rbuffer.bird_hand_controller[i].position;
		bird_hand_reply_structure.bird_hand_controller[i].motion_in_progress
				= edp_ecp_rbuffer.bird_hand_controller[i].motion_in_progress;
	}
	bird_hand_reply_structure.contact = edp_ecp_rbuffer.contact;
	if (bird_hand_reply_data_request_port.is_new_request()) {
		bird_hand_reply_data_request_port.set(bird_hand_reply_structure);
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

