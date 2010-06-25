/*
 * generator/ecp_g_epos.cc
 *
 *Author: yoyek
 */

#include "ecp_g_epos.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

//constructor with parameters: task and time to sleep [s]
epos::epos(common::task::task& _ecp_task) :
	generator(_ecp_task) {
	//	if (the_robot) the_robot->communicate_with_edp = false; //do not communicate with edp


	epos_low_level_command_data_port = the_robot->port_manager.get_port<lib::epos_low_level_command> (EPOS_LOW_LEVEL_COMMAND_DATA_PORT);
	epos_reply_data_request_port = the_robot->port_manager.get_request_port<lib::epos_reply> (EPOS_REPLY_DATA_REQUEST_PORT);

	epos_gen_parameters_data_port = the_robot->port_manager.get_port<lib::epos_gen_parameters> (EPOS_GEN_PARAMETERS_DATA_PORT);

}

void epos::create_ecp_mp_reply() {

}

void epos::get_mp_ecp_command() {
	memcpy(&mp_ecp_epos_gen_parameters_structure,
			ecp_t.mp_command.ecp_next_state.mp_2_ecp_next_state_string,
			sizeof(mp_ecp_epos_gen_parameters_structure));

	printf("aaaaa: %lf\n", mp_ecp_epos_gen_parameters_structure.dm[4]);
}

bool epos::first_step() {

	// parameters copying
	get_mp_ecp_command();

	ecp_t.sr_ecp_msg->message("epos first_step");

	//epos_data_port_command_structure.da[3] = 3.13;
	ecp_edp_epos_gen_parameters_structure
			= mp_ecp_epos_gen_parameters_structure;
	epos_gen_parameters_data_port->set(ecp_edp_epos_gen_parameters_structure);
	epos_reply_data_request_port->set_request();

	return true;
}

bool epos::next_step() {
	ecp_t.sr_ecp_msg->message("epos next_step");

	if (epos_reply_data_request_port->get(edp_ecp_epos_reply_structure) == mrrocpp::lib::NewData) {

		std::stringstream ss(std::stringstream::in | std::stringstream::out);
		ss << "licznik: "
				<< edp_ecp_epos_reply_structure.epos_controller[3].position;

		ecp_t.sr_ecp_msg->message(ss.str().c_str());

	}

	bool motion_in_progress = false;

	for (int i = 0; i < 6; i++) {
		if (edp_ecp_epos_reply_structure.epos_controller[i].motion_in_progress
				== true) {
			motion_in_progress = true;
			break;
		}
	}

	if (motion_in_progress) {
		epos_reply_data_request_port->set_request();
		return true;
	} else {
		return false;
	}

}

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp

