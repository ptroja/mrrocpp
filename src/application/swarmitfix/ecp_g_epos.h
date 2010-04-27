/*
 * generator/ecp_g_epos.h
 *
 *Author: yoyek
 */

#ifndef ECP_G_EPOS_H_
#define ECP_G_EPOS_H_

#include <time.h>
#include "ecp/common/generator/ecp_generator.h"
#include "ecp_mp_t_swarmitfix.h"
#include "lib/data_port_headers/epos.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

class epos : public common::generator::generator
{
	private:
		double waittime;		//seconds to wait
		timespec sleeptime;		//structure for nanosleep function
		timespec acttime;
		timespec prevtime;
		timespec starttime;
		lib::epos_gen_parameters mp_ecp_epos_params;

		lib::single_thread_port<lib::epos_low_level_command>* epos_low_level_command_data_port;
		lib::single_thread_request_port<lib::epos_reply>* epos_reply_data_request_port;
		lib::epos_low_level_command epos_data_port_command_structure;
		lib::epos_reply epos_data_port_reply_structure;



	public:
		void create_ecp_mp_reply();
		void get_mp_ecp_command();


		epos(common::task::task& _ecp_task, double=1);		//constructor
		bool first_step();		//first step generation
		bool next_step();			//next step generation
		void init_time(double=1);	//initialize time
};

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif /* ECP_G_SLEEP_H_ */
