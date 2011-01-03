/*
 * generator/ecp_g_epos.h
 *
 *Author: yoyek
 */

#ifndef ECP_G_EPOS_H_
#define ECP_G_EPOS_H_

#include "base/ecp/ecp_generator.h"
#include "robot/epos/dp_epos.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

class epos_cubic : public common::generator::generator
{
private:

	//	lib::epos_gen_parameters mp_ecp_epos_gen_parameters_structure;

	lib::single_thread_port <lib::epos::epos_cubic_command> * epos_cubic_command_data_port;

	lib::single_thread_request_port <lib::epos::epos_reply> * epos_reply_data_request_port;

public:
	void create_ecp_mp_reply();
	void get_mp_ecp_command();

	epos_cubic(common::task::task& _ecp_task); //constructor
	bool first_step(); //first step generation
	bool next_step(); //next step generation

};

class epos_trapezoidal : public common::generator::generator
{
private:

	//	lib::epos_gen_parameters mp_ecp_epos_gen_parameters_structure;

	lib::single_thread_port <lib::epos::epos_trapezoidal_command> * epos_trapezoidal_command_data_port;

	lib::single_thread_request_port <lib::epos::epos_reply> * epos_reply_data_request_port;

public:
	void create_ecp_mp_reply();
	void get_mp_ecp_command();

	epos_trapezoidal(common::task::task& _ecp_task); //constructor
	bool first_step(); //first step generation
	bool next_step(); //next step generation

};

class epos_operational : public common::generator::generator
{
private:

	//	lib::epos_gen_parameters mp_ecp_epos_gen_parameters_structure;

	lib::single_thread_port <lib::epos::epos_operational_command> * epos_operational_command_data_port;

	lib::single_thread_request_port <lib::epos::epos_reply> * epos_reply_data_request_port;

public:
	void create_ecp_mp_reply();
	void get_mp_ecp_command();

	epos_operational(common::task::task& _ecp_task); //constructor
	bool first_step(); //first step generation
	bool next_step(); //next step generation

};

class epos_brake : public common::generator::generator
{
private:

	//	lib::epos_gen_parameters mp_ecp_epos_gen_parameters_structure;

	lib::single_thread_port <bool> * epos_brake_command_data_port;

public:
	void create_ecp_mp_reply();
	void get_mp_ecp_command();

	epos_brake(common::task::task& _ecp_task); //constructor
	bool first_step(); //first step generation
	bool next_step(); //next step generation

};

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif /* ECP_G_SLEEP_H_ */
