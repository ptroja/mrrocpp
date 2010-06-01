// -------------------------------------------------------------------------
//                              mp_task.cc
//
// MP Master Process - methods
//
// -------------------------------------------------------------------------
// Funkcje do konstruowania procesow MP

#include <stdio.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <signal.h>
#include <stdarg.h>
#include <errno.h>
#include <stdlib.h>
#include <unistd.h>
#include <fstream>
#include <string.h>

#include <boost/foreach.hpp>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/srlib.h"
#include "lib/datastr.h"

#include "mp/mp.h"
#include "mp/generator/mp_g_common.h"
#include "mp/generator/mp_g_delay_ms_condition.h"

#include "mp/robot/mp_r_conveyor.h"
#include "mp/robot/mp_r_irp6ot_m.h"
#include "mp/robot/mp_r_irp6p_m.h"
#include "mp/robot/mp_r_irp6_mechatronika.h"
#include "mp/robot/mp_r_speaker.h"
#include "mp/robot/mp_r_polycrank.h"
#include "mp/robot/mp_r_bird_hand.h"
#include "mp/robot/mp_r_spkm.h"
#include "mp/robot/mp_r_smb.h"
#include "mp/robot/mp_r_irp6ot_tfg.h"
#include "mp/robot/mp_r_irp6p_tfg.h"
#include "mp/robot/mp_r_shead.h"

#if defined(USE_MESSIP_SRR)
#include "messip_dataport.h"
#endif

namespace mrrocpp {
namespace mp {
namespace task {

using namespace std;

#if !defined(USE_MESSIP_SRR)
name_attach_t* task::mp_pulse_attach = NULL;
#else
messip_channel_t* task::mp_pulse_attach = NULL;
#endif

// KONSTRUKTORY
task::task(lib::configurator &_config) :
	ecp_mp::task::task(_config), ui_opened(false), ui_new_pulse(false) {
	// initialize communication with other processes
	initialize_communication();

	// Utworzenie listy robotow, powolanie procesow ECP i nawiazanie komunikacji z nimi
	create_robots();
}

task::~task() {
	// Remove (kill) all ECP from the container
	BOOST_FOREACH(const common::robot_pair_t & robot_node, robot_m)
{	delete robot_node.second;
}

// TODO: check for error
if(mp_pulse_attach) {
#if !defined(USE_MESSIP_SRR)
	name_detach(mp_pulse_attach, 0);
#else
	messip::port_delete(mp_pulse_attach);
#endif
	mp_pulse_attach = NULL;
}
}

void task::stop_and_terminate()
{
sr_ecp_msg->message("To terminate MP click STOP icon");
try {
	wait_for_stop ();
} catch(common::MP_main_error & e) {
	exit(EXIT_FAILURE);
}
terminate_all (robot_m);
}

// powolanie robotow w zaleznosci od zawartosci pliku konfiguracyjnego
void task::create_robots()
{
/*
 * this is necessary to first create robot and then assign it to robot_m
 * reason: mp_robot() constructor uses this map (by calling
 * mp_task::mp_wait_for_name_open() so needs the map to be in
 * a consistent state
 */
robot::robot* created_robot;

// ROBOT CONVEYOR
if (config.value<int>("is_conveyor_active", UI_SECTION)) {
	created_robot = new robot::conveyor (*this);
	robot_m[lib::ROBOT_CONVEYOR] = created_robot;
}

// ROBOT SPEAKER
if (config.value<int>("is_speaker_active", UI_SECTION)) {
	created_robot = new robot::speaker (*this);
	robot_m[lib::ROBOT_SPEAKER] = created_robot;
}

// ROBOT IRP6_MECHATRONIKA
if (config.value<int>("is_irp6_mechatronika_active", UI_SECTION)) {
	created_robot = new robot::irp6_mechatronika (*this);
	robot_m[lib::ROBOT_IRP6_MECHATRONIKA] = created_robot;
}

// ROBOT POLYCRANK
if (config.value<int>("is_polycrank_active", UI_SECTION)) {
	created_robot = new robot::polycrank (*this);
	robot_m[lib::ROBOT_POLYCRANK] = created_robot;
}

// ROBOT BIRD_HAND
if (config.value<int>("is_bird_hand_active", UI_SECTION)) {
	created_robot = new robot::bird_hand (*this);
	robot_m[lib::ROBOT_BIRD_HAND] = created_robot;
}

// ROBOT SPKM
if (config.value<int>("is_spkm_active", UI_SECTION)) {
	created_robot = new robot::spkm (*this);
	robot_m[lib::ROBOT_SPKM] = created_robot;
}

// ROBOT SMB
if (config.value<int>("is_smb_active", UI_SECTION)) {
	created_robot = new robot::smb (*this);
	robot_m[lib::ROBOT_SMB] = created_robot;
}

// ROBOT SHEAD
if (config.value<int>("is_shead_active", UI_SECTION)) {
	created_robot = new robot::shead (*this);
	robot_m[lib::ROBOT_SHEAD] = created_robot;
}

// ROBOT IRP6OT_TFG
if (config.value<int>("is_irp6ot_tfg_active", UI_SECTION)) {
	created_robot = new robot::irp6ot_tfg (*this);
	robot_m[lib::ROBOT_IRP6OT_TFG] = created_robot;
}

// ROBOT IRP6P_TFG
if (config.value<int>("is_irp6p_tfg_active", UI_SECTION)) {
	created_robot = new robot::irp6p_tfg (*this);
	robot_m[lib::ROBOT_IRP6P_TFG] = created_robot;
}

// ROBOT IRP6OT_M
if (config.value<int>("is_irp6ot_m_active", UI_SECTION)) {
	created_robot = new robot::irp6ot_m (*this);
	robot_m[lib::ROBOT_IRP6OT_M] = created_robot;
}

// ROBOT IRP6P_M
if (config.value<int>("is_irp6p_m_active", UI_SECTION)) {
	created_robot = new robot::irp6p_m (*this);
	robot_m[lib::ROBOT_IRP6P_M] = created_robot;
}

// ROBOT_ELECTRON
if (config.value<int>("is_electron_robot_active", UI_SECTION)) {
	created_robot = new robot::robot (lib::ROBOT_ELECTRON, "[ecp_electron]", *this);
	robot_m[lib::ROBOT_ELECTRON] = created_robot;
}

// ROBOT_SPEECHRECOGNITION
if (config.value<int>("is_speechrecognition_active", UI_SECTION)) {
	created_robot = new robot::robot (lib::ROBOT_SPEECHRECOGNITION, "[ecp_speechrecognition]", *this);
	robot_m[lib::ROBOT_SPEECHRECOGNITION] = created_robot;
}

// ROBOT_FESTIVAL
if (config.value<int>("is_festival_active", UI_SECTION)) {
	created_robot = new robot::robot (lib::ROBOT_FESTIVAL, "[ecp_festival]", *this);
	robot_m[lib::ROBOT_FESTIVAL] = created_robot;
}
}

// metody do obslugi najczesniej uzywanych generatorow
void task::set_next_playerpos_goal (lib::robot_name_t robot_l, const lib::playerpos_goal_t &goal)
{
// setting the next ecps state
generator::set_next_ecps_state mp_snes_gen(*this);

mp_snes_gen.robot_m[robot_l] = robot_m[robot_l];

mp_snes_gen.configure(goal);

mp_snes_gen.Move();
}

// metody do obslugi najczesniej uzywanych generatorow
void task::set_next_ecps_state (int l_state, int l_variant, const char* l_string, int str_len, int number_of_robots, ... )
{
// setting the next ecps state
generator::set_next_ecps_state mp_snes_gen (*this);

va_list arguments; // A place to store the list of arguments
lib::robot_name_t robot_l;

va_start ( arguments, number_of_robots ); // Initializing arguments to store all values after num
for ( int x = 0; x < number_of_robots; x++ ) // Loop until all numbers are added
{
	robot_l = (lib::robot_name_t) (va_arg ( arguments, int )); // Adds the next value in argument list to sum.
	mp_snes_gen.robot_m[robot_l] = robot_m[robot_l];
}
va_end ( arguments ); // Cleans up the list

mp_snes_gen.configure (l_state, l_variant, l_string, str_len);

mp_snes_gen.Move();
}

// delay MP replacement
void task::wait_ms (int _ms_delay) // zamiast delay
{
generator::delay_ms_condition mp_ds_ms (*this, _ms_delay);

mp_ds_ms.Move();
}

// send_end_motion
void task::send_end_motion_to_ecps (int number_of_robots, ... )
{
generator::send_end_motion_to_ecps mp_semte_gen (*this);

va_list arguments; // A place to store the list of arguments
lib::robot_name_t robot_l;

va_start ( arguments, number_of_robots ); // Initializing arguments to store all values after num
for ( int x = 0; x < number_of_robots; x++ ) // Loop until all numbers are added
{
	robot_l = (lib::robot_name_t) (va_arg ( arguments, int )); // Adds the next value in argument list to sum.
	mp_semte_gen.robot_m[robot_l] = robot_m[robot_l];
}
va_end ( arguments ); // Cleans up the list

mp_semte_gen.Move();
}

// send_end_motion
void task::send_end_motion_to_ecps (int number_of_robots, lib::robot_name_t *properRobotsSet)
{
generator::send_end_motion_to_ecps mp_semte_gen (*this);

lib::robot_name_t robot_l;

for ( int x = 0; x < number_of_robots; x++ ) // Loop until all numbers are added
{
	robot_l = properRobotsSet[x]; // Adds the next value in argument list to sum.
	mp_semte_gen.robot_m[robot_l] = robot_m[robot_l];
}

mp_semte_gen.Move();
}

void task::run_extended_empty_gen (bool activate_trigger, int number_of_robots, ... )
{
generator::extended_empty mp_ext_empty_gen (*this);

va_list arguments; // A place to store the list of arguments
lib::robot_name_t robot_l;

va_start ( arguments, number_of_robots ); // Initializing arguments to store all values after num
for ( int x = 0; x < number_of_robots; x++ ) // Loop until all numbers are added
{
	robot_l = (lib::robot_name_t) (va_arg ( arguments, int )); // Adds the next value in argument list to sum.
	mp_ext_empty_gen.robot_m[robot_l] = robot_m[robot_l];
}
va_end ( arguments ); // Cleans up the list

mp_ext_empty_gen.configure (activate_trigger);

mp_ext_empty_gen.Move();
}

void task::run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots
(int number_of_robots_to_move, int number_of_robots_to_wait_for_task_termin, ... )
{
// CZYNNOSCI WSTEPNE
// utworzenie zbiorow robotow robots_to_move i robots_to_wait_for_task_termination
common::robots_t robots_to_move, robots_to_wait_for_task_termination;
common::robots_t robots_to_move_tmp, robots_to_wait_for_task_termination_tmp;

// powolanie generatora i jego konfiguracja
generator::extended_empty mp_ext_empty_gen (*this);
mp_ext_empty_gen.configure (false);

// na podstawie argumentow wywolania biezacej metody
va_list arguments; // A place to store the list of arguments
lib::robot_name_t robot_l;

// przypisanie robotow do zbiorow robots_to_move i robots_to_wait_for_task_termination, eliminacja robotow ktorych nie ma w systemie
va_start ( arguments, number_of_robots_to_wait_for_task_termin);
// najpierw zbior robots_to_move...
for ( int x = 0; x < number_of_robots_to_move; x++ ) // Loop until all numbers are added
{
	robot_l = (lib::robot_name_t) (va_arg ( arguments, int )); // Adds the next value in argument list to sum.

	if (robot_m.count(robot_l) == 0)
	{
		sr_ecp_msg->message ("run_..._for_set_of_robots_... usunieto nadmiarowe roboty");
	} else {
		robots_to_move[robot_l] = robot_m[robot_l];
	}
}
// ...potem zbior robots_to_wait_for_task_termination
for ( int x = 0; x < number_of_robots_to_wait_for_task_termin; x++ ) // Loop until all numbers are added
{
	robot_l = (lib::robot_name_t) (va_arg ( arguments, int )); // Adds the next value in argument list to sum.
	if (robot_m.count(robot_l) == 0)
	{
		sr_ecp_msg->message ("run_..._for_set_of_robots_... usunieto nadmiarowe roboty 2");
	} else {
		robots_to_wait_for_task_termination[robot_l] = robot_m[robot_l];
	}
}
va_end ( arguments ); // Cleans up the list

// sprawdzenie czy zbior robots_to_wait_for_task_termination nie zawiera robotow, ktorych nie ma w zbiorze robots_to_move

BOOST_FOREACH(const common::robot_pair_t & robot_node, robots_to_wait_for_task_termination) {

	common::robots_t::iterator robots_map_iter = robots_to_move.find(robot_node.first);

	if (robots_map_iter == robots_to_move.end()) {
		sr_ecp_msg->message (lib::SYSTEM_ERROR, 0, "run_ext_empty_gen_for_set_of_robots_... wrong execution arguments");
		throw common::MP_main_error(lib::SYSTEM_ERROR, 0);
	}
}

// GLOWNA PETLA

do {
	// aktualizacja ziorow robotow i sprawdzenie czy zbior robots_to_wait_for_task_termination nie jest juz pusty
	// wtedy wyjscie z petli

	//	if (debug_tmp) printf(" run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots 1\n");
	// przygotowanie zapasowych list robotow
	robots_to_move_tmp.clear();
	robots_to_wait_for_task_termination_tmp.clear();

	robots_to_move_tmp = robots_to_move;
	robots_to_wait_for_task_termination_tmp = robots_to_wait_for_task_termination;

	// sprawdzenie zbioru robots_to_move
	BOOST_FOREACH(const common::robot_pair_t & robot_node, robots_to_move_tmp) {
		if (robot_node.second->ecp_reply_package.reply == lib::TASK_TERMINATED ) {
			//	if (debug_tmp) robot_m_iterator->second->printf_state("1 ");
			robots_to_move.erase (robot_node.first);
		}
	}

	// sprawdzenie zbioru robots_to_wait_for_task_termination
	BOOST_FOREACH(const common::robot_pair_t & robot_node, robots_to_wait_for_task_termination_tmp) {
		if (robot_node.second->ecp_reply_package.reply == lib::TASK_TERMINATED ) {
			//	if (debug_tmp) robot_m_iterator->second->printf_state("2 ");
			robots_to_wait_for_task_termination.erase (robot_node.first);
		}
	}

	// sprawdzenie czy zbior robots_to_wait_for_task_termination jest pusty.
	// Jesli tak wyjscie z petli i w konsekwencji wyjscie z calej metody
	if (robots_to_wait_for_task_termination.empty())
	break;

	// przypisanie generatorowi mp_ext_empty_gen zbioru robots_to_move
	mp_ext_empty_gen.robot_m = robots_to_move;

	//	if (debug_tmp) printf("PRZED MOVE run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots 1\n");
	// uruchomienie generatora
	mp_ext_empty_gen.Move();

	//		if (debug_tmp) printf("ZA MOVE move run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots 1\n");
}while (true);
}

void task::run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots
(int number_of_robots_to_move, int number_of_robots_to_wait_for_task_termin, lib::robot_name_t *robotsToMove, lib::robot_name_t *robotsWaitingForTaskTermination)
{
// CZYNNOSCI WSTEPNE
// utworzenie zbiorow robotow robots_to_move i robots_to_wait_for_task_termination
common::robots_t robots_to_move, robots_to_wait_for_task_termination;
common::robots_t robots_to_move_tmp, robots_to_wait_for_task_termination_tmp;

// powolanie generatora i jego konfiguracja
generator::extended_empty mp_ext_empty_gen (*this);
mp_ext_empty_gen.configure (false);

// na podstawie argumentow wywolania biezacej metody
//va_list arguments;    // A place to store the list of arguments
lib::robot_name_t robot_l;

// przypisanie robotow do zbiorow robots_to_move i robots_to_wait_for_task_termination, eliminacja robotow ktorych nie ma w systemie
//va_start ( arguments, number_of_robots_to_wait_for_task_termin);
// najpierw zbior robots_to_move...
for ( int x = 0; x < number_of_robots_to_move; x++ ) // Loop until all numbers are added
{
	robot_l = robotsToMove[x]; // Adds the next value in argument list to sum.

	if (robot_m.count(robot_l) == 0)
	{
		sr_ecp_msg->message ("run_..._for_set_of_robots_... usunieto nadmiarowe roboty");
	} else {
		robots_to_move[robot_l] = robot_m[robot_l];
	}
}
// ...potem zbior robots_to_wait_for_task_termination
for ( int x = 0; x < number_of_robots_to_wait_for_task_termin; x++ ) // Loop until all numbers are added
{
	robot_l = robotsWaitingForTaskTermination[x]; // Adds the next value in argument list to sum.
	if (robot_m.count(robot_l) == 0)
	{
		sr_ecp_msg->message ("run_..._for_set_of_robots_... usunieto nadmiarowe roboty 2");
	} else {
		robots_to_wait_for_task_termination[robot_l] = robot_m[robot_l];
	}
}
//va_end ( arguments );              // Cleans up the list

// sprawdzenie czy zbior robots_to_wait_for_task_termination nie zawiera robotow, ktorych nie ma w zbiorze robots_to_move

BOOST_FOREACH(const common::robot_pair_t & robot_node, robots_to_wait_for_task_termination) {

	common::robots_t::iterator robots_map_iter = robots_to_move.find(robot_node.first);

	if (robots_map_iter == robots_to_move.end()) {
		sr_ecp_msg->message (lib::SYSTEM_ERROR, 0, "run_ext_empty_gen_for_set_of_robots_... wrong execution arguments");
		throw common::MP_main_error(lib::SYSTEM_ERROR, (uint64_t) 0);
	}
}

// GLOWNA PETLA

do {
	// aktualizacja ziorow robotow i sprawdzenie czy zbior robots_to_wait_for_task_termination nie jest juz pusty
	// wtedy wyjscie z petli

	//	if (debug_tmp) printf(" run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots 1\n");
	// przygotowanie zapasowych list robotow
	robots_to_move_tmp.clear();
	robots_to_wait_for_task_termination_tmp.clear();

	robots_to_move_tmp = robots_to_move;
	robots_to_wait_for_task_termination_tmp = robots_to_wait_for_task_termination;

	// sprawdzenie zbioru robots_to_move
	BOOST_FOREACH(const common::robot_pair_t & robot_node, robots_to_move_tmp) {
		if (robot_node.second->ecp_reply_package.reply == lib::TASK_TERMINATED ) {
			//	if (debug_tmp) robot_m_iterator->second->printf_state("1 ");
			robots_to_move.erase (robot_node.first);
		}
	}

	// sprawdzenie zbioru robots_to_wait_for_task_termination
	BOOST_FOREACH(const common::robot_pair_t & robot_node, robots_to_wait_for_task_termination_tmp) {
		if (robot_node.second->ecp_reply_package.reply == lib::TASK_TERMINATED ) {
			//	if (debug_tmp) robot_m_iterator->second->printf_state("2 ");
			robots_to_wait_for_task_termination.erase (robot_node.first);
		}
	}

	// sprawdzenie czy zbior robots_to_wait_for_task_termination jest pusty.
	// Jesli tak wyjscie z petli i w konsekwencji wyjscie z calej metody
	if (robots_to_wait_for_task_termination.empty())
	break;

	// przypisanie generatorowi mp_ext_empty_gen zbioru robots_to_move
	mp_ext_empty_gen.robot_m = robots_to_move;

	//	if (debug_tmp) printf("PRZED MOVE run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots 1\n");
	// uruchomienie generatora
	mp_ext_empty_gen.Move();
	//		if (debug_tmp) printf("ZA MOVE move run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots 1\n");
}while (true);
// koniec petli
}

int task::wait_for_name_open(void)
{
#if !defined(USE_MESSIP_SRR)
/* Do your MsgReceive's here now with the chid */
while (1) {
	struct _pulse msg;
	struct _msg_info info;

	int rcvid = MsgReceive(mp_pulse_attach->chid, &msg, sizeof(msg), &info);

	if (rcvid == -1) {/* Error condition, exit */
		int e = errno;
		perror("MP: MsgReceivePulse()");
		throw common::MP_main_error(lib::SYSTEM_ERROR, e);
	}

	if (rcvid == 0) {/* Pulse received */
		switch (msg.code) {
			case _PULSE_CODE_DISCONNECT:
			/*
			 * A client disconnected all its connections (called
			 * name_close() for each name_open() of our name) or
			 * terminated
			 */
			ConnectDetach(msg.scoid);
			break;
			case _PULSE_CODE_UNBLOCK:
			/*
			 * REPLY blocked client wants to unblock (was hit by
			 * a signal or timed out).  It's up to you if you
			 * reply now or later.
			 */
			break;
			default:
			/*
			 * A pulse sent by one of your processes or a
			 * _PULSE_CODE_COIDDEATH or _PULSE_CODE_THREADDEATH
			 * from the kernel?
			 */

			if (ui_opened && ui_scoid == msg.scoid) {
				ui_new_pulse = true;
				ui_pulse_code = msg.code;
				// continue; ?
			}

			BOOST_FOREACH(const common::robot_pair_t & robot_node, robot_m) {
				if (robot_node.second->opened && robot_node.second->scoid == msg.scoid) {
					robot_node.second->new_pulse = true;
					robot_node.second->pulse_code = msg.code;
					//						fprintf(stderr, "robot %s pulse %d\n", lib::toString(robot_node.second->robot_name).c_str(), robot_node.second->pulse_code);
				}
			}

			break;
		}
		continue;
	}

	/* name_open() sends a connect message, must EOK this */
	if (msg.type == _IO_CONNECT ) {
		MsgReply( rcvid, EOK, NULL, 0 );
		return info.scoid;
	}

	/* Some other QNX IO message was received; reject it */
	if (msg.type > _IO_BASE && msg.type <= _IO_MAX ) {
		MsgError( rcvid, ENOSYS );
		continue;
	}

	/* A message (presumable ours) received, handle */
	fprintf(stderr, "MP: unexpected message received\n");
	MsgReply(rcvid, ENOSYS, 0, 0);

	throw common::MP_main_error(lib::SYSTEM_ERROR, 0);
}
#else
while(1) {
	int32_t type, subtype;
	int rcvid = messip::port_receive_pulse(mp_pulse_attach, type, subtype);

	if (rcvid == -1) {
		int e = errno;
		perror("MP: messip::port_receive_pulse()");
		throw common::MP_main_error(lib::SYSTEM_ERROR, e);
	} else if (rcvid >= 0) {
		fprintf(stderr, "MP: unexpected message received\n");
		throw common::MP_main_error(lib::SYSTEM_ERROR, 0);
	} else if (rcvid == MESSIP_MSG_NOREPLY) {
		// handle pulse
		if (ui_opened && ui_scoid == mp_pulse_attach->lastmsg_sockfd) {
			ui_new_pulse = true;
			ui_pulse_code = type;
		}

		BOOST_FOREACH(const common::robot_pair_t & robot_node, robot_m) {
			if (robot_node.second->opened && robot_node.second->scoid == mp_pulse_attach->lastmsg_sockfd) {
				robot_node.second->new_pulse = true;
				robot_node.second->pulse_code = type;
			}
		}
	} else if (rcvid == MESSIP_MSG_CONNECTING) {
		return mp_pulse_attach->lastmsg_sockfd;
	}
}
#endif
}

bool task::check_and_optional_wait_for_new_pulse (WAIT_FOR_NEW_PULSE_MODE process_type, const RECEIVE_PULSE_MODE desired_wait_mode)
{
RECEIVE_PULSE_MODE current_wait_mode(NONBLOCK);

bool desired_pulse_found = false;

// checking of already registered pulses
if ((process_type == NEW_ECP_PULSE) || (process_type == NEW_UI_OR_ECP_PULSE)) {
	BOOST_FOREACH(const common::robot_pair_t & robot_node, robot_m) {
		if ((robot_node.second->new_pulse) && !(robot_node.second->new_pulse_checked)) {
			desired_pulse_found = true;
		}
	}
}

if ((process_type == NEW_UI_PULSE) || (process_type == NEW_UI_OR_ECP_PULSE)) {
	if (ui_new_pulse) {
		desired_pulse_found = true;
	}
}

bool exit_from_while = false;

while(!exit_from_while) {
#if !defined(USE_MESSIP_SRR)
	if (current_wait_mode == NONBLOCK) {
		// like creceive in QNX4
		if (TimerTimeout(CLOCK_REALTIME, _NTO_TIMEOUT_RECEIVE, NULL, NULL, NULL ) == -1) {
			int e = errno;
			perror("MP: TimerTimeout()");
			throw common::MP_main_error(lib::SYSTEM_ERROR, e);
		}
	}

	/* Do your MsgReceive's here now with the chid */
	struct _pulse msg;

	int rcvid = MsgReceivePulse(mp_pulse_attach->chid, &msg, sizeof(msg), NULL);

	if (rcvid == -1) {/* Error condition, exit */
		if (errno == ETIMEDOUT) {
			if (desired_wait_mode == BLOCK && !desired_pulse_found) {
				current_wait_mode = BLOCK;
				continue;
			} else {
				exit_from_while = true;
			}
			continue;
		}
		int e = errno;
		perror("MP: MsgReceivePulse()");
		throw common::MP_main_error(lib::SYSTEM_ERROR, e);
	} else if (rcvid == 0) {/* Pulse received */
		switch (msg.code) {
			case _PULSE_CODE_DISCONNECT:
			/*
			 * A client disconnected all its connections (called
			 * name_close() for each name_open() of our name) or
			 * terminated
			 */
			ConnectDetach(msg.scoid);
			break;
			case _PULSE_CODE_UNBLOCK:
			/*
			 * REPLY blocked client wants to unblock (was hit by
			 * a signal or timed out).  It's up to you if you
			 * reply now or later.
			 */
			break;
			default:
			/*
			 * A pulse sent by one of your processes or a
			 * _PULSE_CODE_COIDDEATH or _PULSE_CODE_THREADDEATH
			 * from the kernel?
			 */

			if (ui_opened && ui_scoid == msg.scoid) {
				ui_new_pulse = true;
				ui_pulse_code = msg.code;
				if ((process_type == NEW_UI_PULSE) || (process_type == NEW_UI_OR_ECP_PULSE)) {
					desired_pulse_found = true;
					if (current_wait_mode == BLOCK) {
						exit_from_while = true;
					}
				}
				continue;
			}

			BOOST_FOREACH(const common::robot_pair_t & robot_node, robot_m) {
				if (robot_node.second->opened && robot_node.second->scoid == msg.scoid) {
					robot_node.second->new_pulse = true;
					robot_node.second->pulse_code = msg.code;
					if (clock_gettime(CLOCK_REALTIME, &(robot_node.second->pulse_receive_time)) == -1) {
						perror("clock_gettime()");
					}

					if ((process_type == NEW_ECP_PULSE) || (process_type == NEW_UI_OR_ECP_PULSE)) {
						if (!(robot_node.second->new_pulse_checked)) {
							desired_pulse_found = true;
							if (current_wait_mode == BLOCK) {
								exit_from_while = true;
							}
						}
					}
					// can we get out of this loop?
				}
			}
		}
	} else {
		/* message was not expected here;
		 * this should not happend if using MsgReceivePulse, but we want to be sure
		 */
		MsgError( rcvid, ENOSYS );
		fprintf(stderr, "MP: unexpected message received\n");
		throw common::MP_main_error(lib::SYSTEM_ERROR, 0);
	}
#else
	int32_t type, subtype;
	int rcvid = messip::port_receive_pulse(mp_pulse_attach, type, subtype,
			(current_wait_mode == BLOCK) ? MESSIP_NOTIMEOUT : 0);

	if (rcvid == -1) {
		int e = errno;
		perror("MP: messip::port_receive_pulse()");
		throw common::MP_main_error(lib::SYSTEM_ERROR, e);
	} else if (rcvid == MESSIP_MSG_TIMEOUT) {
		if (desired_wait_mode == BLOCK && !desired_pulse_found) {
			current_wait_mode = BLOCK;
		} else {
			exit_from_while = true;
		}
		continue;
	} else if (rcvid >= 0) {
		fprintf(stderr, "MP: unexpected message received\n");
		throw common::MP_main_error(lib::SYSTEM_ERROR, 0);
	} else if (rcvid == MESSIP_MSG_NOREPLY) {
		// handle pulse
		if (ui_opened && ui_scoid == mp_pulse_attach->lastmsg_sockfd) {
			ui_new_pulse = true;
			ui_pulse_code = type;
			if ((process_type == NEW_UI_PULSE) || (process_type == NEW_UI_OR_ECP_PULSE)) {
				desired_pulse_found = true;
				if (current_wait_mode == BLOCK) {
					exit_from_while = true;
				}
			}
			fprintf(stderr, "new UI pulse type %d received\n", type);
			continue;
		}

		BOOST_FOREACH(const common::robot_pair_t & robot_node, robot_m) {
			if (robot_node.second->opened && robot_node.second->scoid == mp_pulse_attach->lastmsg_sockfd) {
				robot_node.second->new_pulse = true;
				robot_node.second->pulse_code = type;
				if ((process_type == NEW_ECP_PULSE) || (process_type == NEW_UI_OR_ECP_PULSE)) {
					if (!(robot_node.second->new_pulse_checked)) {
						desired_pulse_found = true;
						if (current_wait_mode == BLOCK) {
							exit_from_while = true;
						}
					}
				}
				// can we get out of this loop?
				fprintf(stderr, "new %s pulse type %d received\n", lib::toString(robot_node.second->robot_name).c_str(), type);
			}
		}
	}
#endif
}

return desired_pulse_found;
}

//
// funkcja odbierajaca pulsy z UI lub ECP wykorzystywana w MOVE
//
// intended use:
//
// when the_generator.wait_for_ECP_pulse is set
//     1) block for ECP pulse and react to UI pulses
// otherwise
//     2) peak for UI pulse and eventually react for in in pause/resume/stop/trigger cycle
void task::mp_receive_ui_or_ecp_pulse (common::robots_t & _robot_m, generator::generator& the_generator )
{
enum MP_STATE_ENUM
{
	MP_STATE_RUNNING,
	MP_STATE_PAUSED
}mp_state = MP_STATE_RUNNING;

bool ui_exit_from_while = false;
bool ecp_exit_from_while = (the_generator.wait_for_ECP_pulse) ? false : true;

// 0 0 -> enter
// 0 1 -> enter
// 1 0 -> enter
// 1 1 -> not enter
while (!(ui_exit_from_while && ecp_exit_from_while)) {

	WAIT_FOR_NEW_PULSE_MODE from_who;
	RECEIVE_PULSE_MODE block_mode;

	if (mp_state == MP_STATE_RUNNING) {
		// check for UI pulse or block for UI/ECP pulse
		from_who = NEW_UI_OR_ECP_PULSE;
		block_mode = ecp_exit_from_while ? NONBLOCK : BLOCK;
	} else {
		// block for UI resume/stop pulse
		from_who = NEW_UI_PULSE;
		block_mode = BLOCK;
	}

	if(check_and_optional_wait_for_new_pulse (from_who, block_mode)) {
		// UI Pulse arrived
		if (ui_new_pulse) {

			ui_new_pulse = false;

			switch (ui_pulse_code) {
				case MP_STOP:
				terminate_all (_robot_m);
				throw common::MP_main_error(lib::NON_FATAL_ERROR, ECP_STOP_ACCEPTED);
				case MP_PAUSE:
				mp_state = MP_STATE_PAUSED;
				request_communication_with_robots(_robot_m);
				ui_exit_from_while = false;
				continue;
				default:
				break;
			}

			if (mp_state == MP_STATE_PAUSED) {// oczekujemy na resume
				if (ui_pulse_code == MP_RESUME) { // odebrano resume
					mp_state = MP_STATE_RUNNING;
					ui_exit_from_while = true;
				}
			} else {
				if (ui_pulse_code == MP_TRIGGER) { // odebrano trigger
					ui_exit_from_while = true;
					the_generator.trigger = true;
					// 2 ponizsze linie po dodaniu prawdziwej sporadycznej synchrnozniacji
					ui_exit_from_while = true;
					ecp_exit_from_while = true;
				}
			}
			continue;
		} else {
			if (mp_state == MP_STATE_RUNNING) {
				ui_exit_from_while = true;
			}
		}

		if (the_generator.wait_for_ECP_pulse) {
			BOOST_FOREACH(const common::robot_pair_t & robot_node, robot_m) {
				if ((robot_node.second->new_pulse) && !(robot_node.second->new_pulse_checked)) {
					robot_node.second->new_pulse_checked = true;
					//	 if (debug_tmp)	printf("wait_for_ECP_pulse r: %d, pc: %d\n", robot_node.first, robot_node.second->ui_pulse_code);
					ecp_exit_from_while = true;
				}
			}
		} else {
			ecp_exit_from_while = true;
		}
	} else {
		if (mp_state == MP_STATE_RUNNING) {
			ui_exit_from_while = true;
		}
	}
}
}

// -------------------------------------------------------------------
// inicjacja polaczen, rejestracja nazwy MP, odszukanie UI, SR by Y&W
// -------------------------------------------------------------------
void task::initialize_communication()
{
const std::string sr_net_attach_point = config.return_attach_point_name(lib::configurator::CONFIG_SERVER, "sr_attach_point", UI_SECTION);
const std::string mp_attach_point = config.return_attach_point_name(lib::configurator::CONFIG_SERVER, "mp_attach_point");

// Obiekt do komuniacji z SR
sr_ecp_msg = new lib::sr_ecp(lib::MP, mp_attach_point, sr_net_attach_point, true); // Obiekt do komuniacji z SR
sh_msg = new lib::sr_ecp(lib::MP, mp_attach_point, sr_net_attach_point, false); // Obiekt do komuniacji z SR

const std::string mp_pulse_attach_point = config.return_attach_point_name(lib::configurator::CONFIG_SERVER, "mp_pulse_attach_point");

// Rejestracja kanalu dla pulsow z procesu UI
if(!mp_pulse_attach) {
#if !defined(USE_MESSIP_SRR)
	if ((mp_pulse_attach = name_attach(NULL, mp_pulse_attach_point.c_str(), NAME_FLAG_ATTACH_GLOBAL)) == NULL)
#else
	if ((mp_pulse_attach = messip::port_create(mp_pulse_attach_point)) == NULL)
#endif
	{
		uint64_t e = errno; // kod bledu systemowego
		perror("Failed to attach UI Pulse chanel for Master Process");
		sr_ecp_msg->message (lib::SYSTEM_ERROR, e, "MP: Failed to attach UI Pulse channel");

		throw common::MP_main_error(lib::SYSTEM_ERROR, 0);
	}
}

ui_scoid = wait_for_name_open();
ui_opened = true;
}
// -------------------------------------------------------------------

void task::wait_for_start ()
{
// Oczekiwanie na zlecenie START od UI

while (1) {
	check_and_optional_wait_for_new_pulse(NEW_UI_PULSE, BLOCK);

	if (ui_new_pulse) {
		ui_new_pulse = false;
		if (ui_pulse_code == MP_START) {
			break;
		}
	}
}

sr_ecp_msg->message("MP user program is running");
}
// ------------------------------------------------------------------------


void task::wait_for_stop (void)
{
// Oczekiwanie na zlecenie STOP od UI

sr_ecp_msg->message("To terminate user program click STOP icon");

while (1) {
	check_and_optional_wait_for_new_pulse (NEW_UI_PULSE, BLOCK);

	if (ui_new_pulse) {
		ui_new_pulse = false;
		if (ui_pulse_code == MP_STOP) {
			break;
		}
	}
}
}
// ------------------------------------------------------------------------


// ------------------------------------------------------------------------

void task::start_all (const common::robots_t & _robot_m)
{
// Wystartowanie wszystkich ECP

// przepisanie mapy robotow do skomunikowania na wersje tymczasowa
common::robots_t robots_m_tmp = _robot_m;

while (1) {

	BOOST_FOREACH(const common::robot_pair_t & robot_node, robots_m_tmp) {
		if (robot_node.second->new_pulse ) {
			if (robot_node.second->pulse_code == ECP_WAIT_FOR_START) {
				robot_node.second->new_pulse = false;
				robot_node.second->new_pulse_checked = false;
				robot_node.second->start_ecp();
				//					fprintf(stderr, "starting %s robot\n", lib::toString(robot_node.second->robot_name).c_str());
			} else {
				printf("phase 2 bledny kod pulsu w start_all\n");
				throw common::MP_main_error(lib::NON_FATAL_ERROR, INVALID_ECP_PULSE_IN_MP_START_ALL);
			}
			robots_m_tmp.erase(robot_node.first);
		}
	}

	if (robots_m_tmp.empty()) {
		break;
	} else {
		assert(0);
		check_and_optional_wait_for_new_pulse (NEW_ECP_PULSE, BLOCK);
	}
}
//	BOOST_FOREACH(const common::robot_pair_t & robot_node, _robot_m) {
//		fprintf(stderr, "task::start_all check: robot %s new_pulse %d new_pulse_checked %d pulse_code %d\n",
//				lib::toString(robot_node.second->robot_name).c_str(),
//				robot_node.second->new_pulse, robot_node.second->new_pulse_checked, robot_node.second->pulse_code);
//	}
}
// ------------------------------------------------------------------------


// ------------------------------------------------------------------------
void task::execute_all (const common::robots_t & _robot_m)
{
// Wystartowanie wszystkich ECP
// do przepisania wg http://www.thescripts.com/forum/thread62378.html

common::robots_t robots_m_tmp;

// przepisanie mapy robotow do skomunikowania na wersje tymczasowa
BOOST_FOREACH(const common::robot_pair_t & robot_node, _robot_m) {
	if (robot_node.second->communicate) {
		if ((robot_node.second->mp_command.command == lib::STOP) || (robot_node.second->mp_command.command == lib::END_MOTION) ||
				(robot_node.second->mp_command.command == lib::NEXT_STATE) || (robot_node.second->continuous_coordination) || (robot_node.second->new_pulse))
		{
			robots_m_tmp.insert(robot_node);
		}
	}

}

request_communication_with_robots(robots_m_tmp);

while (1) {

	BOOST_FOREACH(const common::robot_pair_t & robot_node, robots_m_tmp) {
		// komunikujemy sie tylko z aktywnymi robotami
		if (robot_node.second->new_pulse) {
			if ((robot_node.second->pulse_code == ECP_WAIT_FOR_COMMAND) ||
					(robot_node.second->pulse_code == ECP_WAIT_FOR_NEXT_STATE)) {
				robot_node.second->new_pulse = false;
				robot_node.second->new_pulse_checked = false;
				robot_node.second->execute_motion();
			} else {
				throw common::MP_main_error(lib::NON_FATAL_ERROR, INVALID_ECP_PULSE_IN_MP_EXECUTE_ALL);
			}
			robots_m_tmp.erase(robot_node.first);
		}
	}

	if (robots_m_tmp.empty()) {
		break;
	} else {
		check_and_optional_wait_for_new_pulse (NEW_ECP_PULSE, BLOCK);
	}
}
}
// ------------------------------------------------------------------------


// ------------------------------------------------------------------------
void task::request_communication_with_robots (const common::robots_t & _robot_m)
{
// Zatrzymanie wszystkich ECP

// przepisanie mapy robotow do skomunikowania na wersje tymczasowa
BOOST_FOREACH(const common::robot_pair_t & robot_node, _robot_m) {
	// wysylamy zadanie komunikacji do ECP o ile ECP jeszcze sie nie zglosilo
	robot_node.second->send_pulse_to_ecp(MP_TO_ECP_COMMUNICATION_REQUEST);

}
}

// ------------------------------------------------------------------------
void task::terminate_all (const common::robots_t & _robot_m)
{
// Zatrzymanie wszystkich ECP

request_communication_with_robots(_robot_m);

// przepisanie mapy robotow do skomunikowania na wersje tymczasowa
common::robots_t robots_m_tmp = _robot_m;

while (1) {

	BOOST_FOREACH(const common::robot_pair_t & robot_node, robots_m_tmp) {

		if (robot_node.second->new_pulse) {
			//if ((robot_m_iterator->second->ui_pulse_code == ECP_WAIT_FOR_STOP) || (robot_m_iterator->second->ui_pulse_code == ECP_WAIT_FOR_COMMAND))
			if (1) {
				robot_node.second->new_pulse = false;
				robot_node.second->new_pulse_checked = false;
				robot_node.second->terminate_ecp();
			} else {
				printf("phase 2 bledny kod pulsu w terminate_all\n");
				throw common::MP_main_error(lib::NON_FATAL_ERROR, INVALID_ECP_PULSE_IN_MP_TERMINATE_ALL);
			}
			robots_m_tmp.erase(robot_node.first);
		}
	}

	if (robots_m_tmp.empty()) {
		break;
	} else {
		check_and_optional_wait_for_new_pulse (NEW_ECP_PULSE, BLOCK);
	}
}
}
// ------------------------------------------------------------------------

} // namespace task
} // namespace mp
} // namespace mrrocpp
