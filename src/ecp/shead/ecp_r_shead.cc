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

#include "ecp/shead/ecp_r_shead.h"

namespace mrrocpp {
namespace ecp {
namespace shead {

robot::robot(lib::configurator &_config, lib::sr_ecp &_sr_ecp) :
	shead_head_soldification_data_port(SHEAD_HEAD_SOLIDIFICATION_DATA_PORT),
			shead_vacuum_activation_data_port(SHEAD_VACUUM_ACTIVATION_DATA_PORT),
			shead_reply_data_request_port(SHEAD_VACUUM_ACTIVATION_DATA_PORT),
			ecp_robot(lib::ROBOT_SHEAD, SHEAD_NUM_OF_SERVOS, EDP_SHEAD_SECTION, _config, _sr_ecp), kinematics_manager()
{
	//  Stworzenie listy dostepnych kinematyk.
	create_kinematic_models_for_given_robot();
}
;
robot::robot(common::task::task& _ecp_object) :
	shead_head_soldification_data_port(SHEAD_HEAD_SOLIDIFICATION_DATA_PORT),
			shead_vacuum_activation_data_port(SHEAD_VACUUM_ACTIVATION_DATA_PORT),
			shead_reply_data_request_port(SHEAD_VACUUM_ACTIVATION_DATA_PORT),
			ecp_robot(lib::ROBOT_SHEAD, SHEAD_NUM_OF_SERVOS, EDP_SHEAD_SECTION, _ecp_object), kinematics_manager()
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

void robot::add_data_ports()
{
	port_manager.add_port(&shead_head_soldification_data_port);
	port_manager.add_port(&shead_vacuum_activation_data_port);
	port_manager.add_port(&shead_reply_data_request_port);

}

void robot::clear_data_ports()
{
	shead_head_soldification_data_port.clear_new_data_flag();
	shead_vacuum_activation_data_port.clear_new_data_flag();
	shead_reply_data_request_port.clear_new_request_flag();
	shead_reply_data_request_port.clear_new_data_flag();
}

void robot::create_command()
{

	int new_data_counter;
	bool is_new_data;
	bool is_new_request;

	sr_ecp_msg.message("create_command");

	is_new_data = false;

	if (shead_head_soldification_data_port.get(shead_head_soldification_structure) == mrrocpp::lib::NewData) {
		ecp_command.instruction.set_type = ARM_DEFINITION;

		// generator command interpretation
		// narazie proste przepisanie

		ecp_edp_cbuffer.variant = lib::SHEAD_CBUFFER_HEAD_SOLIDIFICATION;

		ecp_edp_cbuffer.head_solidification = shead_head_soldification_structure;

		if (is_new_data) {
			throw ecp_robot::ECP_error(lib::NON_FATAL_ERROR, INVALID_COMMAND_TO_EDP);
		} else {
			is_new_data = true;
		}
	}

	if (shead_vacuum_activation_data_port.get(shead_vacuum_activation_structure) == mrrocpp::lib::NewData) {
		ecp_command.instruction.set_type = ARM_DEFINITION;

		// generator command interpretation
		// narazie proste przepisanie

		ecp_edp_cbuffer.variant = lib::SHEAD_CBUFFER_VACUUM_ACTIVATION;

		ecp_edp_cbuffer.vacuum_activation = shead_vacuum_activation_structure;

		if (is_new_data) {
			throw ecp_robot::ECP_error(lib::NON_FATAL_ERROR, INVALID_COMMAND_TO_EDP);
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


	shead_reply_structure = edp_ecp_rbuffer.reply;

	if (shead_reply_data_request_port.is_new_request()) {
		shead_reply_data_request_port.set(shead_reply_structure);
	}

}

} // namespace smb
} // namespace ecp
} // namespace mrrocpp

