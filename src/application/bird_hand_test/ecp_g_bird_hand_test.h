/*
 * generator/ecp_g_bird_hand_test.h
 *
 *Author: yoyek
 */

#ifndef ECP_G_BIRD_HAND_H_
#define ECP_G_BIRD_HAND_H_

#include <time.h>
#include "ecp/common/generator/ecp_generator.h"
#include "lib/data_port_headers/bird_hand.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

class bird_hand: public common::generator::generator {
private:
	// zadawanie nastaw regulatorow
	lib::single_thread_port<lib::bird_hand_command>
			*bird_hand_command_data_port;
	lib::bird_hand_command bird_hand_command_structure;

	// zadawanie parametrow konfiguracji
	lib::single_thread_port<lib::bird_hand_configuration>
			*bird_hand_configuration_command_data_port;
	lib::bird_hand_configuration bird_hand_configuration_command_structure;

	// odbieranie statusu robota
	lib::single_thread_request_port<lib::bird_hand_status>
			*bird_hand_status_reply_data_request_port;
	lib::bird_hand_status bird_hand_status_reply_structure;

	// odczytanie parametrow konfiguracji
	lib::single_thread_request_port<lib::bird_hand_configuration>
			*bird_hand_configuration_reply_data_request_port;
	lib::bird_hand_configuration bird_hand_configuration_reply_structure;

public:
	void create_ecp_mp_reply();
	void get_mp_ecp_command();

	bird_hand(common::task::task& _ecp_task); //constructor
	bool first_step(); //first step generation
	bool next_step(); //next step generation

};

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif /* ECP_G_SLEEP_H_ */
