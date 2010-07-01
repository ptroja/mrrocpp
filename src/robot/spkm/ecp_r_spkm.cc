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

#include "robot/spkm/ecp_r_spkm.h"

namespace mrrocpp {
namespace ecp {
namespace spkm {

robot::robot(lib::configurator &_config, lib::sr_ecp &_sr_ecp) :
	ecp_robot(lib::ROBOT_SPKM, SPKM_NUM_OF_SERVOS, EDP_SPKM_SECTION, _config, _sr_ecp), kinematics_manager(),
			epos_cubic_command_data_port(lib::EPOS_CUBIC_COMMAND_DATA_PORT, port_manager),
			epos_trapezoidal_command_data_port(lib::EPOS_TRAPEZOIAL_COMMAND_DATA_PORT, port_manager),
			epos_operational_command_data_port(lib::EPOS_OPERATIONAL_COMMAND_DATA_PORT, port_manager),
			epos_reply_data_request_port(lib::EPOS_REPLY_DATA_REQUEST_PORT, port_manager)

{

	//  Stworzenie listy dostepnych kinematyk.
	create_kinematic_models_for_given_robot();

}

robot::robot(common::task::task& _ecp_object) :
	ecp_robot(lib::ROBOT_SPKM, SPKM_NUM_OF_SERVOS, EDP_SPKM_SECTION, _ecp_object), kinematics_manager(),
			epos_cubic_command_data_port(lib::EPOS_CUBIC_COMMAND_DATA_PORT, port_manager),
			epos_trapezoidal_command_data_port(lib::EPOS_TRAPEZOIAL_COMMAND_DATA_PORT, port_manager),
			epos_operational_command_data_port(lib::EPOS_OPERATIONAL_COMMAND_DATA_PORT, port_manager),
			epos_reply_data_request_port(lib::EPOS_REPLY_DATA_REQUEST_PORT, port_manager)

{

	//  Stworzenie listy dostepnych kinematyk.
	create_kinematic_models_for_given_robot();
}

void robot::create_command()
{

	//	int new_data_counter;
	bool is_new_data;
	bool is_new_request;

	sr_ecp_msg.message("create_command");

	is_new_data = false;

	if (epos_cubic_command_data_port.get(epos_cubic_command_structure) == mrrocpp::lib::NewData) {
		ecp_command.instruction.set_type = ARM_DEFINITION;
		// generator command interpretation
		// narazie proste przepisanie

		ecp_edp_cbuffer.variant = lib::SPKM_CBUFFER_EPOS_CUBIC_COMMAND;

		ecp_edp_cbuffer.epos_cubic_command_structure = epos_cubic_command_structure;

		if (is_new_data) {
			throw ecp_robot::ECP_error(lib::NON_FATAL_ERROR, INVALID_COMMAND_TO_EDP);
		} else {
			is_new_data = true;
		}
	}

	if (epos_trapezoidal_command_data_port.get(epos_trapezoidal_command_structure) == mrrocpp::lib::NewData) {
		ecp_command.instruction.set_type = ARM_DEFINITION;
		// generator command interpretation
		// narazie proste przepisanie

		ecp_edp_cbuffer.variant = lib::SPKM_CBUFFER_EPOS_TRAPEZOIDAL_COMMAND;

		ecp_edp_cbuffer.epos_trapezoidal_command_structure = epos_trapezoidal_command_structure;

		if (is_new_data) {
			throw ecp_robot::ECP_error(lib::NON_FATAL_ERROR, INVALID_COMMAND_TO_EDP);
		} else {
			is_new_data = true;
		}
	}

	if (epos_operational_command_data_port.get(epos_operational_command_structure) == mrrocpp::lib::NewData) {
		ecp_command.instruction.set_type = ARM_DEFINITION;
		// generator command interpretation
		// narazie proste przepisanie

		ecp_edp_cbuffer.variant = lib::SPKM_CBUFFER_EPOS_OPERATIONAL_COMMAND;

		ecp_edp_cbuffer.epos_operational_command_structure = epos_operational_command_structure;

		if (is_new_data) {
			throw ecp_robot::ECP_error(lib::NON_FATAL_ERROR, INVALID_COMMAND_TO_EDP);
		} else {
			is_new_data = true;
		}
	}
	/*
	 if (epos_gen_parameters_data_port.get(epos_gen_parameters_structure) == mrrocpp::lib::NewData) {
	 ecp_command.instruction.set_type = ARM_DEFINITION;
	 // generator command interpretation
	 // narazie proste przepisanie

	 ecp_edp_cbuffer.variant = lib::SPKM_CBUFFER_EPOS_GEN_PARAMETERS;

	 ecp_edp_cbuffer.epos_gen_parameters_structure = epos_gen_parameters_structure;

	 if (is_new_data) {
	 throw ecp_robot::ECP_error(lib::NON_FATAL_ERROR, INVALID_COMMAND_TO_EDP);
	 } else {
	 is_new_data = true;
	 }
	 }
	 */
	is_new_request = epos_reply_data_request_port.is_new_request();

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

	if (epos_reply_data_request_port.is_new_request()) {
		ecp_command.instruction.get_type = ARM_DEFINITION; // arm - ORYGINAL
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
	for (int i = 0; i < 6; i++) {
		epos_reply_structure.epos_controller[i].position = edp_ecp_rbuffer.epos_controller[i].position;
		epos_reply_structure.epos_controller[i].motion_in_progress
				= edp_ecp_rbuffer.epos_controller[i].motion_in_progress;
	}
	epos_reply_structure.contact = edp_ecp_rbuffer.contact;
	if (epos_reply_data_request_port.is_new_request()) {
		epos_reply_data_request_port.set(epos_reply_structure);
	}

}

// Stworzenie modeli kinematyki dla robota IRp-6 na postumencie.
void robot::create_kinematic_models_for_given_robot(void)
{
	// Stworzenie wszystkich modeli kinematyki.
	add_kinematic_model(new kinematics::spkm::kinematic_model_spkm());
	// Ustawienie aktywnego modelu.
	set_kinematic_model(0);
}

} // namespace spkm
} // namespace ecp
} // namespace mrrocpp

