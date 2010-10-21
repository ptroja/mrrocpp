/*
 * generator/ecp_g_epos.cc
 *
 *Author: yoyek
 */

#include "base/ecp/ecp_task.h"
#include "base/ecp/ecp_robot.h"
#include "ecp_g_epos.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

//
//
//
// epos_cubic
//
//
//


//constructor with parameters: task and time to sleep [s]
epos_cubic::epos_cubic(common::task::task& _ecp_task) :
	generator(_ecp_task)
{
	//	if (the_robot) the_robot->communicate_with_edp = false; //do not communicate with edp


	epos_cubic_command_data_port
			= the_robot->port_manager.get_port <lib::epos::epos_cubic_command> (lib::epos::EPOS_CUBIC_COMMAND_DATA_PORT);
	epos_reply_data_request_port
			= the_robot->port_manager.get_request_port <lib::epos::epos_reply> (lib::epos::EPOS_REPLY_DATA_REQUEST_PORT);

}

void epos_cubic::create_ecp_mp_reply()
{

}

void epos_cubic::get_mp_ecp_command()
{
	//	memcpy(&mp_ecp_epos_gen_parameters_structure, ecp_t.mp_command.ecp_next_state.mp_2_ecp_next_state_string, sizeof(mp_ecp_epos_gen_parameters_structure));

	//	printf("aaaaa: %lf\n", mp_ecp_epos_gen_parameters_structure.dm[4]);
}

bool epos_cubic::first_step()
{

	// parameters copying
	get_mp_ecp_command();

	sr_ecp_msg.message("epos first_step");

	//epos_data_port_command_structure.da[3] = 3.13;
	epos_reply_data_request_port->set_request();

	return true;
}

bool epos_cubic::next_step()
{
	sr_ecp_msg.message("epos next_step");

	if (epos_reply_data_request_port->get() == mrrocpp::lib::NewData) {

		std::stringstream ss(std::stringstream::in | std::stringstream::out);
		ss << "licznik: " << epos_reply_data_request_port->data.epos_controller[3].position;

		sr_ecp_msg.message(ss.str().c_str());

	}

	bool motion_in_progress = false;

	for (int i = 0; i < 6; i++) {
		if (epos_reply_data_request_port->data.epos_controller[i].motion_in_progress == true) {
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

//
//
//
// epos_trapezoidal
//
//
//


epos_trapezoidal::epos_trapezoidal(common::task::task& _ecp_task) :
	generator(_ecp_task)
{
	//	if (the_robot) the_robot->communicate_with_edp = false; //do not communicate with edp


	epos_trapezoidal_command_data_port
			= the_robot->port_manager.get_port <lib::epos::epos_trapezoidal_command> (lib::epos::EPOS_TRAPEZOIDAL_COMMAND_DATA_PORT);
	epos_reply_data_request_port
			= the_robot->port_manager.get_request_port <lib::epos::epos_reply> (lib::epos::EPOS_REPLY_DATA_REQUEST_PORT);

}

void epos_trapezoidal::create_ecp_mp_reply()
{

}

void epos_trapezoidal::get_mp_ecp_command()
{
	//	memcpy(&mp_ecp_epos_gen_parameters_structure, ecp_t.mp_command.ecp_next_state.mp_2_ecp_next_state_string, sizeof(mp_ecp_epos_gen_parameters_structure));

	//	printf("aaaaa: %lf\n", mp_ecp_epos_gen_parameters_structure.dm[4]);
}

bool epos_trapezoidal::first_step()
{

	// parameters copying
	get_mp_ecp_command();

	sr_ecp_msg.message("epos first_step");

	//epos_data_port_command_structure.da[3] = 3.13;
	epos_reply_data_request_port->set_request();

	return true;
}

bool epos_trapezoidal::next_step()
{
	sr_ecp_msg.message("epos next_step");

	if (epos_reply_data_request_port->get() == mrrocpp::lib::NewData) {

		std::stringstream ss(std::stringstream::in | std::stringstream::out);
		ss << "licznik: " << epos_reply_data_request_port->data.epos_controller[3].position;

		sr_ecp_msg.message(ss.str().c_str());

	}

	bool motion_in_progress = false;

	for (int i = 0; i < 6; i++) {
		if (epos_reply_data_request_port->data.epos_controller[i].motion_in_progress == true) {
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

//
//
//
// epos_operational
//
//
//


epos_operational::epos_operational(common::task::task& _ecp_task) :
	generator(_ecp_task)
{
	//	if (the_robot) the_robot->communicate_with_edp = false; //do not communicate with edp


	epos_operational_command_data_port
			= the_robot->port_manager.get_port <lib::epos::epos_operational_command> (lib::epos::EPOS_OPERATIONAL_COMMAND_DATA_PORT);
	epos_reply_data_request_port
			= the_robot->port_manager.get_request_port <lib::epos::epos_reply> (lib::epos::EPOS_REPLY_DATA_REQUEST_PORT);

}

void epos_operational::create_ecp_mp_reply()
{

}

void epos_operational::get_mp_ecp_command()
{
	//	memcpy(&mp_ecp_epos_gen_parameters_structure, ecp_t.mp_command.ecp_next_state.mp_2_ecp_next_state_string, sizeof(mp_ecp_epos_gen_parameters_structure));

	//	printf("aaaaa: %lf\n", mp_ecp_epos_gen_parameters_structure.dm[4]);
}

bool epos_operational::first_step()
{

	// parameters copying
	get_mp_ecp_command();

	sr_ecp_msg.message("epos first_step");

	//epos_data_port_command_structure.da[3] = 3.13;
	epos_reply_data_request_port->set_request();

	return true;
}

bool epos_operational::next_step()
{
	sr_ecp_msg.message("epos next_step");

	if (epos_reply_data_request_port->get() == mrrocpp::lib::NewData) {

		std::stringstream ss(std::stringstream::in | std::stringstream::out);
		ss << "licznik: " << epos_reply_data_request_port->data.epos_controller[3].position;

		sr_ecp_msg.message(ss.str().c_str());

	}

	bool motion_in_progress = false;

	for (int i = 0; i < 6; i++) {
		if (epos_reply_data_request_port->data.epos_controller[i].motion_in_progress == true) {
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

//
//
//
// epos_brake
//
//
//


epos_brake::epos_brake(common::task::task& _ecp_task) :
	generator(_ecp_task)
{
	//	if (the_robot) the_robot->communicate_with_edp = false; //do not communicate with edp


	epos_brake_command_data_port = the_robot->port_manager.get_port <bool> (lib::epos::EPOS_BRAKE_COMMAND_DATA_PORT);

}

void epos_brake::create_ecp_mp_reply()
{

}

void epos_brake::get_mp_ecp_command()
{
	//	memcpy(&mp_ecp_epos_gen_parameters_structure, ecp_t.mp_command.ecp_next_state.mp_2_ecp_next_state_string, sizeof(mp_ecp_epos_gen_parameters_structure));

	//	printf("aaaaa: %lf\n", mp_ecp_epos_gen_parameters_structure.dm[4]);
}

bool epos_brake::first_step()
{

	return true;
}

bool epos_brake::next_step()
{

	return true;

}

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp

