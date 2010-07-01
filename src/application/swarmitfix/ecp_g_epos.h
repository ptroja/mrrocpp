/*
 * generator/ecp_g_epos.h
 *
 *Author: yoyek
 */

#ifndef ECP_G_EPOS_H_
#define ECP_G_EPOS_H_

#include <time.h>
#include "base/ecp/ecp_generator.h"
#include "lib/data_port_headers/epos.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

class epos_cubic : public common::generator::generator
{
private:

//	lib::epos_gen_parameters mp_ecp_epos_gen_parameters_structure;

	lib::single_thread_port <lib::epos_cubic_command> * epos_cubic_command_data_port;
	lib::epos_cubic_command ecp_edp_cubic_command_structure;



	lib::single_thread_request_port <lib::epos_reply> * epos_reply_data_request_port;
	lib::epos_reply edp_ecp_epos_reply_structure;

public:
	void create_ecp_mp_reply();
	void get_mp_ecp_command();

	epos_cubic(common::task::task& _ecp_task); //constructor
	bool first_step(); //first step generation
	bool next_step(); //next step generation

};

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif /* ECP_G_SLEEP_H_ */
