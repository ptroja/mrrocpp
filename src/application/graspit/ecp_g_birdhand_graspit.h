/*
 * generator/ecp_g_bird_hand_test.h
 *
 *Author: yoyek
 */

#ifndef ECP_G_BIRDHAND_GRASPIT_H_
#define ECP_G_BIRDHAND_GRASPIT_H_

#include "base/ecp/ecp_generator.h"
#include "base/lib/single_thread_port.h"
#include "robot/bird_hand/dp_bird_hand.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

/*!
 * \class bird_hand
 * \brief Constant velocity generator for Bird Hand, position control
 *
 * \author kczajkow
 * \date Jan 05, 2010
 */
class bird_hand : public common::generator::generator
{
private:
	//! zadawanie nastaw regulatorow
	lib::single_thread_port <lib::bird_hand::command> *bird_hand_command_data_port;

	//! zadawanie parametrow konfiguracji
	lib::single_thread_port <lib::bird_hand::configuration> *bird_hand_configuration_command_data_port;

	//! odbieranie statusu robota
	lib::single_thread_request_port <lib::bird_hand::status> *bird_hand_status_reply_data_request_port;

	//! odczytanie parametrow konfiguracji
	lib::single_thread_request_port <lib::bird_hand::configuration> *bird_hand_configuration_reply_data_request_port;

	//! docelowe pozycje palcow
	double des_thumb_f[2];
	double des_index_f[3];
	double des_ring_f[3];

	//! liczba makrokrokow
	int macro_no;
	//! dlugosc ostatniego makrokroku
	int last_step;

	//! maksymalna predkosc [rad/ms]
	const double MAX_V;
	//! czas trwania makrokroku, 1 step = 1ms
	const int STEP_NO;

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
