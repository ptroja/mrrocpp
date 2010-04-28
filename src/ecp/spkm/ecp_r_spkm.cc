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

#include "ecp/spkm/ecp_r_spkm.h"

namespace mrrocpp {
namespace ecp {
namespace spkm {

robot::robot(lib::configurator &_config, lib::sr_ecp &_sr_ecp) :
	epos_low_level_command_data_port(EPOS_LOW_LEVEL_COMMAND_DATA_PORT),
			epos_gen_parameters_data_port(EPOS_GEN_PARAMETERS_DATA_PORT),
			epos_reply_data_request_port(EPOS_REPLY_DATA_REQUEST_PORT),
			ecp_robot(lib::ROBOT_SPKM, SPKM_NUM_OF_SERVOS, EDP_SPKM_SECTION,
					_config, _sr_ecp), kinematics_manager() {
	add_data_ports();
	//  Stworzenie listy dostepnych kinematyk.
	create_kinematic_models_for_given_robot();

}

robot::robot(common::task::task& _ecp_object) :
	epos_low_level_command_data_port(EPOS_LOW_LEVEL_COMMAND_DATA_PORT),
			epos_gen_parameters_data_port(EPOS_GEN_PARAMETERS_DATA_PORT),
			epos_reply_data_request_port(EPOS_REPLY_DATA_REQUEST_PORT),
			ecp_robot(lib::ROBOT_SPKM, SPKM_NUM_OF_SERVOS, EDP_SPKM_SECTION,
					_ecp_object), kinematics_manager() {
	add_data_ports();
	//  Stworzenie listy dostepnych kinematyk.
	create_kinematic_models_for_given_robot();
}

void robot::add_data_ports() {
	port_manager.add_port(&epos_low_level_command_data_port);
	port_manager.add_port(&epos_gen_parameters_data_port);
	port_manager.add_port(&epos_reply_data_request_port);
}

void robot::create_command() {

	if (epos_low_level_command_data_port.is_new_data()
			&& epos_reply_data_request_port.is_new_request()) {
		ecp_command.instruction.instruction_type = lib::SET_GET;
	} else if (epos_low_level_command_data_port.is_new_data()) {
		ecp_command.instruction.instruction_type = lib::SET;
	} else if (epos_reply_data_request_port.is_new_request()) {
		ecp_command.instruction.instruction_type = lib::GET;
	}

	if (epos_reply_data_request_port.is_new_request()) {
		ecp_command.instruction.get_type = ARM_DEFINITION; // arm - ORYGINAL
	}

	if (epos_low_level_command_data_port.is_new_data()) {
		ecp_command.instruction.set_type = ARM_DEFINITION;
		epos_data_port_command_structure
				= epos_low_level_command_data_port.get();
		// generator command interpretation
		// narazie proste przepisanie

		ecp_edp_cbuffer.variant = lib::SPKM_CBUFFER_EPOS_LOW_LEVEL_COMMAND;

		ecp_edp_cbuffer.epos_data_port_command_structure = epos_data_port_command_structure;


	} else if (epos_gen_parameters_data_port.is_new_data()) {
		ecp_command.instruction.set_type = ARM_DEFINITION;
		epos_data_port_gen_parameters_structure
				= epos_gen_parameters_data_port.get();
		// generator command interpretation
		// narazie proste przepisanie

		ecp_edp_cbuffer.variant = lib::SPKM_CBUFFER_EPOS_GEN_PARAMETERS;

		ecp_edp_cbuffer.epos_data_port_gen_parameters_structure  = epos_data_port_gen_parameters_structure;
	} else

	{
		ecp_edp_cbuffer.variant = lib::SPKM_CBUFFER_NO_ACTION;
	}
	// message serialization
	memcpy(ecp_command.instruction.arm.serialized_command, &ecp_edp_cbuffer,
			sizeof(ecp_edp_cbuffer));
}

void robot::get_reply() {
	// message deserialization
	memcpy(&edp_ecp_rbuffer, reply_package.arm.serialized_reply,
			sizeof(edp_ecp_rbuffer));

	// generator reply generation
	for (int i = 0; i < 6; i++) {
		epos_data_port_reply_structure.position[i]
				= edp_ecp_rbuffer.position[i];
		epos_data_port_reply_structure.motion_in_progress[i]
				= edp_ecp_rbuffer.motion_in_progress[i];
	}
	if (epos_reply_data_request_port.is_new_request()) {
		epos_reply_data_request_port.set(epos_data_port_reply_structure);
	}
}

// Stworzenie modeli kinematyki dla robota IRp-6 na postumencie.
void robot::create_kinematic_models_for_given_robot(void) {
	// Stworzenie wszystkich modeli kinematyki.
	add_kinematic_model(new kinematics::spkm::kinematic_model_spkm());
	// Ustawienie aktywnego modelu.
	set_kinematic_model(0);
}

} // namespace spkm
} // namespace ecp
} // namespace mrrocpp

