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
#include <boost/throw_exception.hpp>
#include <boost/exception/errinfo_errno.hpp>
#include <boost/exception/errinfo_api_function.hpp>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/srlib.h"
#include "lib/datastr.h"
#include "lib/exception.h"

#include "base/mp/mp.h"
#include "base/mp/mp_g_common.h"
#include "base/mp/mp_g_delay_ms_condition.h"

#include "robot/conveyor/mp_r_conveyor.h"
#include "robot/irp6ot_m/mp_r_irp6ot_m.h"
#include "robot/irp6p_m/mp_r_irp6p_m.h"
#include "robot/irp6_mechatronika/mp_r_irp6_mechatronika.h"
#include "robot/speaker/mp_r_speaker.h"
#include "robot/polycrank/mp_r_polycrank.h"
#include "robot/bird_hand/mp_r_bird_hand.h"
#include "robot/irp6ot_tfg/mp_r_irp6ot_tfg.h"
#include "robot/irp6p_tfg/mp_r_irp6p_tfg.h"
#include "robot/shead/mp_r_shead.h"
#include "robot/spkm/mp_r_spkm.h"
#include "robot/smb/mp_r_smb.h"
#include "robot/sarkofag/mp_r_sarkofag.h"

#include "robot/irp6_mechatronika/irp6m_const.h"
#include "robot/irp6ot_m/irp6ot_m_const.h"
#include "robot/irp6ot_tfg/irp6ot_tfg_const.h"
#include "robot/irp6p_m/irp6p_m_const.h"
#include "robot/irp6p_tfg/irp6p_tfg_const.h"
#include "robot/polycrank/polycrank_const.h"
#include "robot/smb/smb_const.h"
#include "robot/spkm/spkm_const.h"
#include "robot/shead/shead_const.h"
#include "robot/speaker/speaker_const.h"
#include "robot/conveyor/conveyor_const.h"
#include "robot/bird_hand/bird_hand_const.h"

#if defined(USE_MESSIP_SRR)
#include "messip_dataport.h"
#endif

namespace mrrocpp {
namespace mp {
namespace task {

using namespace std;

// KONSTRUKTORY
task::task(lib::configurator &_config) :
	ecp_mp::task::task(_config),
	ui_command_buffer(*this, UI_COMMAND_BUFFER)
{
	// initialize communication with other processes
	initialize_communication();

	// Utworzenie listy robotow, powolanie procesow ECP i nawiazanie komunikacji z nimi
	create_robots();
}

task::~task()
{
	// Remove (kill) all ECP from the container
	BOOST_FOREACH(const common::robot_pair_t & robot_node, robot_m)
	{
		delete robot_node.second;
	}
}

void task::stop_and_terminate()
{
	sr_ecp_msg->message("To terminate MP click STOP icon");

	wait_for_stop ();

	terminate_all (robot_m);
}

// powolanie robotow w zaleznosci od zawartosci pliku konfiguracyjnego
void task::create_robots()
{
/*
 * it is necessary to first create robot and then assign it to robot_m
 * reason: mp_robot() constructor uses this map (by calling
 * mp_task::mp_wait_for_name_open() so needs the map to be in
 * a consistent state
 */
robot::robot* created_robot;

// ROBOT CONVEYOR
if (config.value<int>("is_conveyor_active", UI_SECTION)) {
	created_robot = new robot::conveyor (*this);
	robot_m[created_robot->robot_name] = created_robot;
}

// ROBOT SPEAKER
if (config.value<int>("is_speaker_active", UI_SECTION)) {
	created_robot = new robot::speaker (*this);
	robot_m[created_robot->robot_name] = created_robot;
}

// ROBOT IRP6_MECHATRONIKA
if (config.value<int>("is_irp6_mechatronika_active", UI_SECTION)) {
	created_robot = new robot::irp6_mechatronika (*this);
	robot_m[created_robot->robot_name] = created_robot;
}

// ROBOT POLYCRANK
if (config.value<int>("is_polycrank_active", UI_SECTION)) {
	created_robot = new robot::polycrank (*this);
	robot_m[created_robot->robot_name] = created_robot;
}

// ROBOT BIRD_HAND
if (config.value<int>("is_bird_hand_active", UI_SECTION)) {
	created_robot = new robot::bird_hand (*this);
	robot_m[created_robot->robot_name] = created_robot;
}

// ROBOT SPKM
if (config.value<int>("is_spkm_active", UI_SECTION)) {
	created_robot = new robot::spkm (*this);
	robot_m[created_robot->robot_name] = created_robot;
}

// ROBOT SMB
if (config.value<int>("is_smb_active", UI_SECTION)) {
	created_robot = new robot::smb (*this);
	robot_m[created_robot->robot_name] = created_robot;
}

// ROBOT SHEAD
if (config.value<int>("is_shead_active", UI_SECTION)) {
	created_robot = new robot::shead (*this);
	robot_m[created_robot->robot_name] = created_robot;
}

// ROBOT IRP6OT_TFG
if (config.value<int>("is_irp6ot_tfg_active", UI_SECTION)) {
	created_robot = new robot::irp6ot_tfg (*this);
	robot_m[created_robot->robot_name] = created_robot;
}

// ROBOT SARKOFAG
if (config.value<int>("is_sarkofag_active", UI_SECTION)) {
	created_robot = new robot::sarkofag (*this);
	robot_m[created_robot->robot_name] = created_robot;
}

// ROBOT IRP6P_TFG
if (config.value<int>("is_irp6p_tfg_active", UI_SECTION)) {
	created_robot = new robot::irp6p_tfg (*this);
	robot_m[created_robot->robot_name] = created_robot;
}

// ROBOT IRP6OT_M
if (config.value<int>("is_irp6ot_m_active", UI_SECTION)) {
	created_robot = new robot::irp6ot_m (*this);
	robot_m[created_robot->robot_name] = created_robot;
}

// ROBOT IRP6P_M
if (config.value<int>("is_irp6p_m_active", UI_SECTION)) {
	created_robot = new robot::irp6p_m (*this);
	robot_m[created_robot->robot_name] = created_robot;
}

// ROBOT_ELECTRON
if (config.value<int>("is_electron_robot_active", UI_SECTION)) {
	created_robot = new robot::robot (lib::ROBOT_ELECTRON, "[ecp_electron]", *this);
	robot_m[created_robot->robot_name] = created_robot;
}

// ROBOT_SPEECHRECOGNITION
if (config.value<int>("is_speechrecognition_active", UI_SECTION)) {
	created_robot = new robot::robot (lib::ROBOT_SPEECHRECOGNITION, "[ecp_speechrecognition]", *this);
	robot_m[created_robot->robot_name] = created_robot;
}

// ROBOT_FESTIVAL
if (config.value<int>("is_festival_active", UI_SECTION)) {
	created_robot = new robot::robot (lib::ROBOT_FESTIVAL, "[ecp_festival]", *this);
	robot_m[created_robot->robot_name] = created_robot;
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
void task::set_next_ecps_state (const std::string & l_state, int l_variant, const char* l_string, int str_len, int number_of_robots, ... )
{
// setting the next ecps state
generator::set_next_ecps_state mp_snes_gen (*this);

va_list arguments; // A place to store the list of arguments
lib::robot_name_t robot_l;

<<<<<<< HEAD
	va_start ( arguments, number_of_robots ); // Initializing arguments to store all values after num
	for ( int x = 0; x < number_of_robots; x++ ) // Loop until all numbers are added
	{
		robot_l = (lib::robot_name_t) (va_arg ( arguments, char * )); // Adds the next value in argument list to sum.
		mp_snes_gen.robot_m[robot_l] = robot_m[robot_l];
	}
	va_end ( arguments ); // Cleans up the list
=======
va_start ( arguments, number_of_robots ); // Initializing arguments to store all values after num
for ( int x = 0; x < number_of_robots; x++ ) // Loop until all numbers are added
{
	robot_l = (lib::robot_name_t) (va_arg ( arguments, char* )); // Adds the next value in argument list to sum.
	mp_snes_gen.robot_m[robot_l] = robot_m[robot_l];
}
va_end ( arguments ); // Cleans up the list
>>>>>>> origin/master

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

<<<<<<< HEAD
	va_start ( arguments, number_of_robots ); // Initializing arguments to store all values after num
	for ( int x = 0; x < number_of_robots; x++ ) // Loop until all numbers are added
	{
		robot_l = (lib::robot_name_t) (va_arg ( arguments, char * )); // Adds the next value in argument list to sum.
		mp_semte_gen.robot_m[robot_l] = robot_m[robot_l];
	}
	va_end ( arguments ); // Cleans up the list
=======
va_start ( arguments, number_of_robots ); // Initializing arguments to store all values after num
for ( int x = 0; x < number_of_robots; x++ ) // Loop until all numbers are added
{
	robot_l = (lib::robot_name_t) (va_arg ( arguments, char* )); // Adds the next value in argument list to sum.
	mp_semte_gen.robot_m[robot_l] = robot_m[robot_l];
}
va_end ( arguments ); // Cleans up the list
>>>>>>> origin/master

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

<<<<<<< HEAD
	va_start ( arguments, number_of_robots ); // Initializing arguments to store all values after num
	for ( int x = 0; x < number_of_robots; x++ ) // Loop until all numbers are added
	{
		robot_l = (lib::robot_name_t) (va_arg ( arguments, char * )); // Adds the next value in argument list to sum.
		mp_ext_empty_gen.robot_m[robot_l] = robot_m[robot_l];
	}
	va_end ( arguments ); // Cleans up the list
=======
va_start ( arguments, number_of_robots ); // Initializing arguments to store all values after num
for ( int x = 0; x < number_of_robots; x++ ) // Loop until all numbers are added
{
	robot_l = (lib::robot_name_t) (va_arg ( arguments, char* )); // Adds the next value in argument list to sum.
	mp_ext_empty_gen.robot_m[robot_l] = robot_m[robot_l];
}
va_end ( arguments ); // Cleans up the list
>>>>>>> origin/master

mp_ext_empty_gen.configure (activate_trigger);

mp_ext_empty_gen.Move();
}

void task::run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots
(int number_of_robots_to_move, int number_of_robots_to_wait_for_task_termin, ... )
{
<<<<<<< HEAD
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
		robot_l = (lib::robot_name_t) (va_arg ( arguments, char * )); // Adds the next value in argument list to sum.
=======
// CZYNNOSCI WSTEPNE
// utworzenie zbiorow robotow robots_to_move i robots_to_wait_for_task_termination
common::robots_t robots_to_move, robots_to_wait_for_task_termination;
common::robots_t robots_to_move_tmp, robots_to_wait_for_task_termination_tmp;
>>>>>>> origin/master

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
	robot_l = (lib::robot_name_t) (va_arg ( arguments, char* )); // Adds the next value in argument list to sum.

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
	robot_l = (lib::robot_name_t) (va_arg ( arguments, char* )); // Adds the next value in argument list to sum.
	if (robot_m.count(robot_l) == 0)
	{
<<<<<<< HEAD
		robot_l = (lib::robot_name_t) (va_arg ( arguments, char * )); // Adds the next value in argument list to sum.
		if (robot_m.count(robot_l) == 0)
		{
			sr_ecp_msg->message ("run_..._for_set_of_robots_... usunieto nadmiarowe roboty 2");
		} else {
			robots_to_wait_for_task_termination[robot_l] = robot_m[robot_l];
		}
=======
		sr_ecp_msg->message ("run_..._for_set_of_robots_... usunieto nadmiarowe roboty 2");
	} else {
		robots_to_wait_for_task_termination[robot_l] = robot_m[robot_l];
>>>>>>> origin/master
	}
}
va_end ( arguments ); // Cleans up the list

// sprawdzenie czy zbior robots_to_wait_for_task_termination nie zawiera robotow, ktorych nie ma w zbiorze robots_to_move

BOOST_FOREACH(const common::robot_pair_t & robot_node, robots_to_wait_for_task_termination) {

	common::robots_t::iterator robots_map_iter = robots_to_move.find(robot_node.first);

<<<<<<< HEAD
		if (robots_map_iter == robots_to_move.end()) {
			sr_ecp_msg->message (lib::FATAL_ERROR, 0, "run_ext_empty_gen_for_set_of_robots_... wrong execution arguments");
			BOOST_THROW_EXCEPTION(
					lib::exception::Fatal_error()
			);
		}
=======
	if (robots_map_iter == robots_to_move.end()) {
		sr_ecp_msg->message (lib::SYSTEM_ERROR, 0, "run_ext_empty_gen_for_set_of_robots_... wrong execution arguments");
		throw common::MP_main_error(lib::SYSTEM_ERROR, 0);
>>>>>>> origin/master
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

<<<<<<< HEAD
		// sprawdzenie czy zbior robots_to_wait_for_task_termination jest pusty.
		// Jesli tak wyjscie z petli i w konsekwencji wyjscie z calej metody
		if (robots_to_wait_for_task_termination.empty())
			break;
=======
	// sprawdzenie czy zbior robots_to_wait_for_task_termination jest pusty.
	// Jesli tak wyjscie z petli i w konsekwencji wyjscie z calej metody
	if (robots_to_wait_for_task_termination.empty())
	break;
>>>>>>> origin/master

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
            BOOST_THROW_EXCEPTION(
            		lib::exception::Fatal_error()
            );
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
	} while (true);
	// koniec petli
}

// -------------------------------------------------------------------
// inicjacja polaczen, rejestracja nazwy MP, odszukanie UI, SR by Y&W
// -------------------------------------------------------------------
void task::initialize_communication()
{
	const std::string sr_net_attach_point = config.return_attach_point_name(lib::configurator::CONFIG_SERVER, "sr_attach_point", UI_SECTION);
	const std::string mp_attach_point = config.return_attach_point_name(lib::configurator::CONFIG_SERVER, "mp_attach_point");

<<<<<<< HEAD
	// Obiekt do komuniacji z SR
	sr_ecp_msg = new lib::sr_ecp(lib::MP, mp_attach_point, sr_net_attach_point, true); // Obiekt do komuniacji z SR
	sh_msg = new lib::sr_ecp(lib::MP, mp_attach_point, sr_net_attach_point, false); // Obiekt do komuniacji z SR
=======
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
		sr_ecp_msg->message (lib::SYSTEM_ERROR, e, "mp: Failed to attach UI Pulse channel");

		throw common::MP_main_error(lib::SYSTEM_ERROR, 0);
	}
}

ui_scoid = wait_for_name_open();
// unexepected possible resolution of bug 1526
delay(100);
ui_opened = true;
>>>>>>> origin/master
}
// -------------------------------------------------------------------

void task::wait_for_start ()
{
	sr_ecp_msg->message("To terminate user program click STOP icon");

	while(ui_command_buffer.Get() != MP_START) {
		Wait(ui_command_buffer);
	}

	sr_ecp_msg->message("MP user program is running");
}
// ------------------------------------------------------------------------


void task::wait_for_stop (void)
{
	// Oczekiwanie na zlecenie STOP od UI

	sr_ecp_msg->message("To terminate user program click STOP icon");

	while(ui_command_buffer.Get() != MP_STOP) {
		Wait(ui_command_buffer);
	}
}
// ------------------------------------------------------------------------


// ------------------------------------------------------------------------

void task::start_all (const common::robots_t & _robot_m)
{
	fprintf(stderr, "task::start_all\n");
	BOOST_FOREACH(const common::robot_pair_t & robot_node, _robot_m)
	{
		robot_node.second->start_ecp();
	}
}
// ------------------------------------------------------------------------


// ------------------------------------------------------------------------
void task::execute_all (const common::robots_t & _robot_m)
{
	BOOST_FOREACH(const common::robot_pair_t & robot_node, _robot_m)
	{
		if (robot_node.second->communicate) {
			robot_node.second->execute_motion();
		}
	}
}
// ------------------------------------------------------------------------


// ------------------------------------------------------------------------
void task::terminate_all (const common::robots_t & _robot_m)
{
	BOOST_FOREACH(const common::robot_pair_t & robot_node, _robot_m)
	{
		robot_node.second->terminate_ecp();
	}
}
// ------------------------------------------------------------------------

} // namespace task
} // namespace mp
} // namespace mrrocpp
