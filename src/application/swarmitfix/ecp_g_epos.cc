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
epos::epos(common::task::task& _ecp_task, double s) :
	generator(_ecp_task) {
//	communicate_with_edp = false; //do not communicate with edp
	waittime = s * 1000; //wait time[ns] conversting from given seconds to nanoseconds
	sleeptime.tv_nsec = 20000000; //sleep time[ns]
	sleeptime.tv_sec = 0;

	epos_low_level_command_data_port = the_robot->port_manager.get_port<lib::epos_low_level_command> (
			EPOS_LOW_LEVEL_COMMAND_DATA_PORT);
	epos_reply_data_request_port = the_robot->port_manager.get_request_port<lib::epos_reply> (
			EPOS_REPLY_DATA_REQUEST_PORT);

	epos_gen_parameters_data_port = the_robot->port_manager.get_port<lib::epos_gen_parameters> (
			EPOS_GEN_PARAMETERS_DATA_PORT);

}

//allow for later change of a sleep time
void epos::init_time(double s) {
	waittime = s * 1000; //TODO: conversion from seconds to nanoseconds (?!)
}

void epos::create_ecp_mp_reply() {

}

void epos::get_mp_ecp_command() {
	memcpy(&mp_ecp_epos_params,
			ecp_t.mp_command.ecp_next_state.mp_2_ecp_next_state_string,
			sizeof(mp_ecp_epos_params));

	printf("aaaaa: %lf\n", mp_ecp_epos_params.dm[4]);
}

bool epos::first_step() {

	// parameters copying
	get_mp_ecp_command();


	//epos_data_port_command_structure.da[3] = 3.13;
	epos_data_port_gen_parameters_structure = mp_ecp_epos_params;
	epos_gen_parameters_data_port->set(epos_data_port_gen_parameters_structure);
	epos_reply_data_request_port->set_request();

	if (clock_gettime(CLOCK_REALTIME, &acttime) == -1) { //acquiring actual time
		printf("sleep generator: first step time measurement error");
		return false;
	}

	starttime = acttime;
	return false;
}

bool epos::next_step() {
	double diff;

	prevtime = acttime;
	if (clock_gettime(CLOCK_REALTIME, &acttime) == -1) {
		printf("sleep generator: next step time measurement error");
	}

	//difference between consecutive next_steeps, check if the pause button was pressed (difference bigger than 100ms)
	diff = (acttime.tv_sec - prevtime.tv_sec) * 1000 + (acttime.tv_nsec
			- prevtime.tv_nsec) / 1000000;
	if (diff > 100)
		waittime = waittime + diff;

	//difference between start time and actual time, check if wait time already passed
	diff = (acttime.tv_sec - starttime.tv_sec) * 1000 + (acttime.tv_nsec
			- starttime.tv_nsec) / 1000000;
	if (diff > waittime)
		return false;
	else {
		nanosleep(&sleeptime, NULL);
		return true;
	}
}

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp

