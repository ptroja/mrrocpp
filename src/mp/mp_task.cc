// -------------------------------------------------------------------------
//                              mp.cc
//
// MP Master Process - methods
//
// Ostatnia modyfikacja: 2007
// -------------------------------------------------------------------------
// Funkcje do konstruowania procesow MP

#include <stdio.h>
#include <signal.h>
#include <stdarg.h>
#include <errno.h>
#include <stdlib.h>
#include <unistd.h>
#include <fstream>

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"

#include "lib/srlib.h"

#include "mp/mp.h"
#include "mp/mp_r_conveyor.h"
#include "mp/mp_r_irp6_on_track.h"
#include "mp/mp_r_irp6_postument.h"
#include "mp/mp_r_irp6_mechatronika.h"
#include "mp/mp_r_speaker.h"

using namespace std;

// bool debug_tmp=false;

// obsluga sygnalu
void mp_task::catch_signal_in_mp_task(int sig) {
	switch (sig) {
		case SIGTERM:
		case SIGINT:
			kill_all_ECP(robot_m);
			kill_all_VSP(sensor_m);
			sr_ecp_msg->message("MP terminated");
			_exit(EXIT_SUCCESS);
			break;
		case SIGSEGV:
			fprintf(stderr, "Segmentation fault in MP process\n");
			signal(SIGSEGV, SIG_DFL);
			break;
	}
};


name_attach_t* mp_task::mp_trigger_attach = NULL;
name_attach_t* mp_task::mp_attach = NULL;


// mapa wszystkich robotow z iteratorem
map <ROBOT_ENUM, mp_robot*> mp_task::robot_m;


// KONSTRUKTORY
mp_task::mp_task() {
	robot_m.clear();
	sensor_m.clear();

	// dla scheduller'a
	gen_list.clear();
	all_gen_sets_waiting_for_ECP_pulse = false;
	ui_new_pulse = false;
};
mp_task::~mp_task() {

	delete[] mrrocpp_network_path;
};

// powolanie robotow w zaleznosci od zawartosci pliku konfiguracyjnego
bool mp_task::create_robots() {

	mp_robot* robot_tmp; // konieczne bo inaczej konstruktor dziala niepoprawnie w odbieranu pulsu od ECP
	// ROBOT IRP6_ON_TRACK
	if (config->return_int_value("is_irp6_on_track_active", "[ui]")) {
		robot_tmp = new mp_irp6_on_track_robot (this);
		robot_m[ROBOT_IRP6_ON_TRACK] = robot_tmp;
	}

	// ROBOT IRP6_POSTUMENT
	if (config->return_int_value("is_irp6_postument_active", "[ui]")) {
		robot_tmp = new mp_irp6_postument_robot (this);
		robot_m[ROBOT_IRP6_POSTUMENT] = robot_tmp;
	}

	// ROBOT CONVEYOR
	if (config->return_int_value("is_conveyor_active", "[ui]")) {
		robot_tmp = new mp_conveyor_robot (this);
		robot_m[ROBOT_CONVEYOR] = robot_tmp;
	}

	// ROBOT SPEAKER
	if (config->return_int_value("is_speaker_active", "[ui]")) {
		robot_tmp = new mp_speaker_robot (this);
		robot_m[ROBOT_SPEAKER] = robot_tmp;
	}

	// ROBOT IRP6_MECHATRONIKA
	if (config->return_int_value("is_irp6_mechatronika_active", "[ui]")) {
		robot_tmp = new mp_irp6_mechatronika_robot (this);
		robot_m[ROBOT_IRP6_MECHATRONIKA] = robot_tmp;
	}

	return true;

}


// methods for mp template to redefine in concrete classes
void mp_task::task_initialization(void) {
	sr_ecp_msg->message("MP loaded");
};


void mp_task::main_task_algorithm(void) {};



// metody do obslugi najczesniej uzywanych generatorow
bool mp_task::set_next_ecps_state (int l_state, int l_variant, char* l_string, int number_of_robots, ... ) {
	// setting the next ecps state
	mp_set_next_ecps_state_generator mp_snes_gen (*this);

	mp_snes_gen.robot_m.clear();

	va_list arguments;                     // A place to store the list of arguments
	ROBOT_ENUM robot_l;

	va_start ( arguments, number_of_robots );           // Initializing arguments to store all values after num
	for ( int x = 0; x < number_of_robots; x++ )        // Loop until all numbers are added
	{
		robot_l = (ROBOT_ENUM) (va_arg ( arguments, int )); // Adds the next value in argument list to sum.
		mp_snes_gen.robot_m[robot_l] = robot_m[robot_l];
	}
	va_end ( arguments );                  // Cleans up the list

	mp_snes_gen.configure (l_state, l_variant, l_string);
	if (Move ( mp_snes_gen)) {  return true;  }

	return false;
}

// delay MP replacement
bool mp_task::wait_ms (int _ms_delay) // zamiast delay
{
	//	delay (_ms_delay); // temporary

	mp_delay_ms_condition mp_ds_ms (*this, _ms_delay);

	mp_ds_ms.robot_m.clear();

	if (Move (mp_ds_ms)) {  return true;  }

	return false;

}



bool mp_task::send_end_motion_to_ecps (int number_of_robots, ... ) {
	// send_end_motion
	mp_send_end_motion_to_ecps_generator mp_semte_gen (*this);

	mp_semte_gen.robot_m.clear();

	va_list arguments;                     // A place to store the list of arguments
	ROBOT_ENUM robot_l;

	va_start ( arguments, number_of_robots );           // Initializing arguments to store all values after num
	for ( int x = 0; x < number_of_robots; x++ )        // Loop until all numbers are added
	{
		robot_l = (ROBOT_ENUM) (va_arg ( arguments, int )); // Adds the next value in argument list to sum.
		mp_semte_gen.robot_m[robot_l] = robot_m[robot_l];
	}
	va_end ( arguments );                  // Cleans up the list

	if (Move ( mp_semte_gen)) {  return true;  }

	return false;
}


bool mp_task::run_ext_empty_gen (bool activate_trigger, int number_of_robots, ... ) {

	mp_extended_empty_generator mp_ext_empty_gen (*this);

	mp_ext_empty_gen.robot_m.clear();

	va_list arguments;                     // A place to store the list of arguments
	ROBOT_ENUM robot_l;

	va_start ( arguments, number_of_robots );           // Initializing arguments to store all values after num
	for ( int x = 0; x < number_of_robots; x++ )        // Loop until all numbers are added
	{
		robot_l = (ROBOT_ENUM) (va_arg ( arguments, int )); // Adds the next value in argument list to sum.
		mp_ext_empty_gen.robot_m[robot_l] = robot_m[robot_l];
	}
	va_end ( arguments );                  // Cleans up the list

	mp_ext_empty_gen.configure (activate_trigger);

	if (Move ( mp_ext_empty_gen)) {  return true;  }

	return false;
}



bool mp_task::run_ext_empty_gen_for_set_of_robots_and_wait_for_task_termin_mess_of_another_set_of_robots
(int number_of_robots_to_move, int number_of_robots_to_wait_for_task_termin, ... ) {

	// CZYNNOSCI WSTEPNE
	// utworzenie zbiorow robotow robots_to_move i robots_to_wait_for_task_termination
	map <ROBOT_ENUM, mp_robot*> robots_to_move, robots_to_wait_for_task_termination;
	map <ROBOT_ENUM, mp_robot*> robots_to_move_tmp, robots_to_wait_for_task_termination_tmp;
	map <ROBOT_ENUM, mp_robot*>::iterator robots_map_iter;

	// powolanie generatora i jego konfiguracja
	mp_extended_empty_generator mp_ext_empty_gen (*this);
	mp_ext_empty_gen.configure (false);

	// na podstawie argumentow wywolania biezacej metody
	va_list arguments;    // A place to store the list of arguments
	ROBOT_ENUM robot_l;

	// przypisanie robotow do zbiorow robots_to_move i robots_to_wait_for_task_termination, eliminacja robotow ktorych nie ma w systemie
	va_start ( arguments, number_of_robots_to_wait_for_task_termin);
	// najpierw zbior robots_to_move
	for ( int x = 0; x < number_of_robots_to_move; x++ )        // Loop until all numbers are added
	{
		robot_l = (ROBOT_ENUM) (va_arg ( arguments, int )); // Adds the next value in argument list to sum.

		if (robot_m.count(robot_l) == 0) {
			sr_ecp_msg->message ("run_..._for_set_of_robots_... usunieto nadmiarowe roboty");
		} else {
			robots_to_move[robot_l] = robot_m[robot_l];
		}
	}
	// najpierw zbior robots_to_wait_for_task_termination
	for ( int x = 0; x < number_of_robots_to_wait_for_task_termin; x++ )        // Loop until all numbers are added
	{
		robot_l = (ROBOT_ENUM) (va_arg ( arguments, int )); // Adds the next value in argument list to sum.
		if (robot_m.count(robot_l) == 0) {
			sr_ecp_msg->message ("run_..._for_set_of_robots_... usunieto nadmiarowe roboty 2");
		} else {
			robots_to_wait_for_task_termination[robot_l] = robot_m[robot_l];
		}
	}
	va_end ( arguments );              // Cleans up the list

	// sprawdzenie czy zbior robots_to_wait_for_task_termination nie zawiera robotow, ktorych nie ma w zbiorze robots_to_move

	for (map <ROBOT_ENUM, mp_robot*>::iterator robot_m_iterator = robots_to_wait_for_task_termination.begin();
	        robot_m_iterator != robots_to_wait_for_task_termination.end(); robot_m_iterator++) {

		robots_map_iter = robots_to_move.find(robot_m_iterator->first);
		if (robots_map_iter == robots_to_move.end()) {
			sr_ecp_msg->message (SYSTEM_ERROR, 0, "run_ext_empty_gen_for_set_of_robots_... wrong execution arguments");
			throw MP_main_error(SYSTEM_ERROR, (uint64_t) 0);
		}
	}

	// GLOWNA PETLA

	do {
		// aktualizacja ziorow robotow i sprawdzenie czy zbior robots_to_wait_for_task_termination nie jest juz pusty
		// wtedy wyjscie z petli

		//	if (debug_tmp) printf(" run_ext_empty_gen_for_set_of_robots_and_wait_for_task_termin_mess_of_another_set_of_robots 1\n");
		// przygotowanie zapasowych list robotow
		robots_to_move_tmp.clear();
		robots_to_wait_for_task_termination_tmp.clear();

		robots_to_move_tmp = robots_to_move;
		robots_to_wait_for_task_termination_tmp = robots_to_wait_for_task_termination;

		// sprawdzenie zbior robots_to_move
		for (map <ROBOT_ENUM, mp_robot*>::iterator robot_m_iterator = robots_to_move_tmp.begin();
		        robot_m_iterator != robots_to_move_tmp.end(); robot_m_iterator++) {
			if (robot_m_iterator->second->ecp_td.ecp_reply == TASK_TERMINATED  ) {
				//	if (debug_tmp) robot_m_iterator->second->printf_state("1 ");
				robots_to_move.erase (robot_m_iterator->first);
			}
		}

		// sprawdzenie zbioru robots_to_wait_for_task_termination
		for (map <ROBOT_ENUM, mp_robot*>::iterator robot_m_iterator = robots_to_wait_for_task_termination_tmp.begin();
		        robot_m_iterator != robots_to_wait_for_task_termination_tmp.end(); robot_m_iterator++) {
			if (robot_m_iterator->second->ecp_td.ecp_reply == TASK_TERMINATED  ) {
				//	if (debug_tmp) robot_m_iterator->second->printf_state("2 ");
				robots_to_wait_for_task_termination.erase (robot_m_iterator->first);
			}
		}

		// sprawdzenie czy zbior robots_to_wait_for_task_termination jest pusty.
		// Jesli tak wyjscie z petli i w konsekwencji wyjscie z calej metody
		if (robots_to_wait_for_task_termination.empty())	break;

		// przypisanie generatorowi mp_ext_empty_gen zbioru robots_to_move
		mp_ext_empty_gen.robot_m.clear();
		mp_ext_empty_gen.robot_m = robots_to_move;

		//	if (debug_tmp) printf("PRZED MOVE run_ext_empty_gen_for_set_of_robots_and_wait_for_task_termin_mess_of_another_set_of_robots 1\n");
		// uruchomienie generatora
	if (Move ( mp_ext_empty_gen)) {  return true;  }
//		if (debug_tmp) printf("ZA MOVE move run_ext_empty_gen_for_set_of_robots_and_wait_for_task_termin_mess_of_another_set_of_robots 1\n");
	} while (true);
	// koniec petli

	// CZYNNOSCI KONCOWE

	return false;
}



// -------------------------------------------------------------------
// inicjacja polaczen, rejestracja nazwy MP, odszukanie UI, SR by Y&W
// -------------------------------------------------------------------

int mp_task::mp_receive_pulse (mp_receive_pulse_struct_tdef* outputs, MP_RECEIVE_PULS_ENUM tryb) {

	struct sigevent event;
	int  wyjscie = 0;


	event.sigev_notify = SIGEV_UNBLOCK;

	while (!wyjscie) {

		if (tryb == WITH_TIMEOUT) {
			TimerTimeout(CLOCK_REALTIME, _NTO_TIMEOUT_RECEIVE,  &event, NULL, NULL );// by Y zamiast creceive
		}

		outputs->rcvid = MsgReceive (mp_trigger_attach->chid, &(outputs->pulse_msg), sizeof(_pulse_msg), &(outputs->msg_info));


		if (outputs->rcvid == -1) {/* Error condition, exit */

			outputs->e = errno;
			wyjscie++;
			continue;

		}

		if (outputs->rcvid == 0) {/* Pulse received */

			switch (outputs->pulse_msg.hdr.code) {
				case _PULSE_CODE_DISCONNECT:
					printf("mp_receive_pulse puls _PULSE_CODE_DISCONNECT\n");
					/*
					 * A client disconnected all its connections (called
					 * name_close() for each name_open() of our name) or
					 * terminated
					 */
					ConnectDetach(outputs->pulse_msg.hdr.scoid);
					break;
				case _PULSE_CODE_UNBLOCK:
					//	  printf("MP_TRIGGER puls _PULSE_CODE_UNBLOCK\n");
					/*
					 * REPLY blocked client wants to unblock (was hit by
					 * a signal or timed out).  It's up to you if you
					 * reply now or later.
					 */
					break;
				default:
					// 	printf("MP_TRIGGER puls default: %d\n", outputs->pulse_msg.hdr.code);
					/*
					* A pulse sent by one of your processes or a
					* _PULSE_CODE_COIDDEATH or _PULSE_CODE_THREADDEATH
					* from the kernel?
					*/
					break;
			}


			wyjscie++;
			continue;
		}

		if (outputs->rcvid > 0) {
			/* A QNX IO message received, reject */
			// ECP wywolalo name_open
			if (outputs->pulse_msg.hdr.type >= _IO_BASE && outputs->pulse_msg.hdr.type <= _IO_MAX) {
				// 	  printf("w MP_TRIGGER _IO_BASE _IO_MAX %d\n",_IO_CONNECT );
				//  MsgError(rcvid, ENOSYS);
//			  printf("mp_receive_ecp_pulse_return_td name_open: %d, %d\n", info.pid, info.scoid);
				MsgReply (outputs->rcvid, EOK, 0, 0);
				wyjscie++;
				//		ret.rt = false;
				continue;
			}
			/* A message (presumable ours) received, handle */
			printf("mp_receive_ecp_pulse server receive strange message of type: %d\n", outputs->pulse_msg.data);
			MsgReply(outputs->rcvid, EOK, 0, 0);
		}
	}

	return outputs->rcvid;

}



int mp_task::check_and_optional_wait_for_new_pulse (mp_receive_pulse_struct_tdef* outputs,
        WAIT_FOR_NEW_PULSE_ENUM process_mode, MP_RECEIVE_PULS_ENUM desired_wait_mode) {

	int ret;
	bool exit_from_while = false;
	bool desired_pulse_found = false;

	MP_RECEIVE_PULS_ENUM current_wait_mode = WITH_TIMEOUT;

//	 printf("check_and_optional_wait_for_new_pulse start\n");

	// checking of already registered pulses

	if ((process_mode == NEW_ECP_PULSE) || (process_mode == NEW_UI_OR_ECP_PULSE)) {
		for (map <ROBOT_ENUM, mp_robot*>::iterator robot_m_iterator = robot_m.begin();
		        robot_m_iterator != robot_m.end(); robot_m_iterator++) {
			if ((robot_m_iterator->second->new_pulse) && (!(robot_m_iterator->second->robot_new_pulse_checked))) {
				desired_pulse_found = true;
			}
		}
	}

	if ((process_mode == NEW_UI_PULSE) || (process_mode == NEW_UI_OR_ECP_PULSE)) {
		if (ui_new_pulse) {
			desired_pulse_found = true;
		}
	}

	// receiving of new pulses

	while (!exit_from_while) {
		ret = mp_receive_pulse (outputs, current_wait_mode);

		if (ret == -1) {
			if (outputs->e != ETIMEDOUT) {
				// tu ma byc wyjatek
				printf ("Blad msgreceive na kanale ecp_pusle w receive_pending_pulses\n");
			} else {
				if ((desired_wait_mode == WITHOUT_TIMEOUT) && (!(desired_pulse_found))) {
					current_wait_mode = WITHOUT_TIMEOUT;
				} else {
					exit_from_while = true;
				}
				continue;
			}

		} else if (ret == 0) {
//			printf("check_and_optional_wait_for_new_pulse ret == 0\n");
			// wstawiamy informacje o pulsie ktory przyszedl do innego robota
			for (map <ROBOT_ENUM, mp_robot*>::iterator robot_m_iterator = robot_m.begin();
			        robot_m_iterator != robot_m.end(); robot_m_iterator++) {
				if (outputs->pulse_msg.hdr.scoid == robot_m_iterator->second->scoid) {
//					printf("check_and_optional_wait_for_new_pulse w ECP\n");
					robot_m_iterator->second->pulse_code = outputs->pulse_msg.hdr.code;
					robot_m_iterator->second->new_pulse = true;
					if ((process_mode == NEW_ECP_PULSE) || (process_mode == NEW_UI_OR_ECP_PULSE)) {
//					printf("a\n");
						if (!(robot_m_iterator->second->robot_new_pulse_checked)) {
//							printf("b\n");
							desired_pulse_found = true;
							if (current_wait_mode == WITHOUT_TIMEOUT) {
//								printf("c\n");
								exit_from_while = true;
							}
						}
					}
					continue;
				}
			}

			if (outputs->pulse_msg.hdr.scoid == ui_scoid) {
				ui_pulse_code = outputs->pulse_msg.hdr.code;
				ui_new_pulse = true;
				if ((process_mode == NEW_UI_PULSE) || (process_mode == NEW_UI_OR_ECP_PULSE)) {
					desired_pulse_found = true;
					if (current_wait_mode == WITHOUT_TIMEOUT) {
						exit_from_while = true;
					}
				}
				continue;
			}

		} else if (ret > 0) {
			// jesli wlasciwy proces zrobil name_open

		}

	}

	if (desired_pulse_found) ret = 0;

//	 printf("check_and_optional_wait_for_new_pulse end\n");

	return ret;
}


// int mp_task::mp_wait_for_name_open_ecp_pulse(mp_receive_pulse_struct_tdef* outputs, uint32_t nd, pid_t ECP_pid)
int mp_task::mp_wait_for_name_open_ecp_pulse(mp_receive_pulse_struct_tdef* outputs) {

	int ret;
	bool wyjscie = false;

	while (!wyjscie) {
		ret = mp_receive_pulse (outputs, WITHOUT_TIMEOUT);
		// jakis inny robot wyslal puls
		if (ret == -1) {
			// tu ma byc wyjatek
			printf ("Blad msgreceive na kanale ecp_pusle w mp_wait_for_name_open_ecp_pulse\n");
		} else if (ret == 0) {

			// wstawiamy informacje o pulsie ktory przyszedl do innego robota
			for (map <ROBOT_ENUM, mp_robot*>::iterator robot_m_iterator = robot_m.begin();
			        robot_m_iterator != robot_m.end(); robot_m_iterator++) {
				if (outputs->pulse_msg.hdr.scoid == robot_m_iterator->second->scoid) {
					robot_m_iterator->second->pulse_code = outputs->pulse_msg.hdr.code;
					robot_m_iterator->second->new_pulse = true;
					continue;
				}
			}

			if (outputs->pulse_msg.hdr.scoid == ui_scoid) {
				ui_pulse_code = outputs->pulse_msg.hdr.code;
				ui_new_pulse = true;
				continue;
			}

		} else if (ret > 0) {
			// zakladamy ze wlasciwy proces zrobi name_open
			wyjscie = true;

			/*
			// jesli wlasciwy proces zrobil name_open
			if ((ND_NODE_CMP(outputs->msg_info.nd, nd) == 0)&&(outputs->msg_info.pid == ECP_pid)) {
				wyjscie = true;
				continue;
			} else {
				printf ("niewlasciwy proces zrobil name_open na kanale ECP_PULSE\n");
			}
			*/
		}

	}

	return ret;
}



int mp_task::mp_wait_for_ui_name_open() {

	int ret;
	mp_receive_pulse_struct_tdef outputs;
	bool wyjscie = false;

	while (!wyjscie) {
		ret = mp_receive_pulse (&outputs, WITHOUT_TIMEOUT);
		// jakis inny robot wyslal puls
		if (ret == -1) {
			// tu ma byc wyjatek
			printf ("Blad msgreceive na kanale ecp_pusle w mp_wait_for_name_open_ecp_pulse\n");
		} else if (ret == 0) {
			printf ("Blad mp_wait_for_ui_name_open ret = 0\n");
		} else if (ret > 0) {
			// jesli wlasciwy proces zrobil name_open
			ui_scoid = outputs.msg_info.scoid;
			wyjscie = true;
			continue;
		}
	}

	return ret;
}


// funkcja odbierajaca pulsy z UI lub ECP wykorzystywana w MOVE

bool mp_task::mp_receive_ui_or_ecp_pulse
(map <ROBOT_ENUM, mp_robot*>& _robot_m, mp_generator& the_generator ) {

//	static long licznik_w=0;
	enum MP_STATE_ENUM
	{
	    MP_STATE_RUNNING,
	    MP_STATE_PAUSED
	};

	MP_STATE_ENUM mp_state = MP_STATE_RUNNING;
	int rcvid;
	mp_receive_pulse_struct_tdef input;

	bool ui_exit_from_while = false;
	bool ecp_exit_from_while = false;

	if (!(the_generator.wait_for_ECP_pulse)) ecp_exit_from_while = true;
// if (debug_tmp) printf("\np1:\n");
	while (!(ui_exit_from_while && ecp_exit_from_while)) {

// if (debug_tmp)	 printf("qqqq: %d\n", licznik_w++);

		if ((mp_state == MP_STATE_RUNNING) && (ecp_exit_from_while)) {
			//  if (debug_tmp)	printf("1 \n");
			rcvid = check_and_optional_wait_for_new_pulse (&input, NEW_UI_OR_ECP_PULSE, WITH_TIMEOUT);
		} else if ((mp_state == MP_STATE_RUNNING) && (!ecp_exit_from_while)) {
			//  if (debug_tmp)	printf("2 \n");
			rcvid = check_and_optional_wait_for_new_pulse (&input, NEW_UI_OR_ECP_PULSE, WITHOUT_TIMEOUT);
// if (debug_tmp)		printf("3 \n");
		} else if (mp_state == MP_STATE_PAUSED) {
			rcvid = check_and_optional_wait_for_new_pulse (&input, NEW_UI_PULSE, WITHOUT_TIMEOUT);
		}

		if (rcvid == -1) {/* Error condition */

			if (mp_state == MP_STATE_RUNNING) {
				if (input.e != ETIMEDOUT) {// by Y zamiast creceive
					// Blad komunikacji miedzyprocesowej - wyjatek
					perror("Creceive STOP or PAUSE proxy from UI failed ?\n");
					sr_ecp_msg->message(SYSTEM_ERROR, input.e, "MP:Creceive STOP pulse from UI failed");
					throw MP_main_error (SYSTEM_ERROR, (uint64_t) 0);
				} else {
					ui_exit_from_while = true;
					continue;
				}
			} else if (mp_state == MP_STATE_PAUSED) {
				perror("Creceive RESUME proxy from UI failed ?\n");
				sr_ecp_msg->message(SYSTEM_ERROR, input.e, "MP: receive RESUME pulse from UI failed");
				throw MP_main_error (SYSTEM_ERROR, (uint64_t) 0);
			}
		} else if (rcvid == 0) {
// 	   printf("rcvid ==0:\n");
			if (ui_new_pulse) {

				ui_new_pulse = false;
				if (ui_pulse_code == MP_STOP) {
					terminate_all (_robot_m);
					return true;
				}

				if (ui_pulse_code == MP_PAUSE) {
//				 printf("ui_new_pulse MP_PAUSED\n");
					usleep(1000*1000);
					mp_state = MP_STATE_PAUSED;
					ui_exit_from_while = false;
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
					}
				}
				continue;
			} else {

				if (mp_state == MP_STATE_RUNNING) {
					//	printf("ui_exit_from_while MP_STATE_RUNNING\n");
					ui_exit_from_while = true;
				}

			}

			if (the_generator.wait_for_ECP_pulse) {
				//	 if (debug_tmp)		printf(" wait_for_ECP_pulse\n");
				for (map <ROBOT_ENUM, mp_robot*>::iterator robot_m_iterator = robot_m.begin();
				        robot_m_iterator != robot_m.end(); robot_m_iterator++) {
					if ((robot_m_iterator->second->new_pulse) && (!(robot_m_iterator->second->robot_new_pulse_checked))) {
						robot_m_iterator->second->robot_new_pulse_checked = true;
						//	 if (debug_tmp)	printf("wait_for_ECP_pulse r: %d, pc: %d\n", robot_m_iterator->first, robot_m_iterator->second->pulse_code);
						ecp_exit_from_while = true;
					}
				}
			} else {
// if (debug_tmp)			printf("else wait_for_ECP_pulse\n");
				ecp_exit_from_while = true;
			}
			continue;

		} else if (rcvid > 0) {
			printf("MP_TRIGGER server receive strange message of type: \n");
		}
	}
// if (debug_tmp) printf("koniec\n");
	return false;

};


// funkcja odbierajaca pulsy z UI wykorzystywana w MOVE

bool mp_task::mp_receive_ui_pulse (map <ROBOT_ENUM, mp_robot*>& _robot_m, short* trigger ) {

	enum MP_STATE_ENUM
	{
	    MP_STATE_RUNNING,
	    MP_STATE_PAUSED
	};

	MP_STATE_ENUM mp_state = MP_STATE_RUNNING;
	int rcvid;
	mp_receive_pulse_struct_tdef input;

	bool wyjscie = false;

	while (!wyjscie) {
		if (mp_state == MP_STATE_RUNNING) {
			rcvid = check_and_optional_wait_for_new_pulse (&input, NEW_UI_PULSE, WITH_TIMEOUT);
		} else {
			rcvid = check_and_optional_wait_for_new_pulse (&input, NEW_UI_PULSE, WITHOUT_TIMEOUT);
		}

		if (rcvid == -1) {// Error condition
			if (mp_state == MP_STATE_RUNNING) {
				if (input.e != ETIMEDOUT) {// by Y zamiast creceive
					// Blad komunikacji miedzyprocesowej - wyjatek
					perror("Creceive STOP or PAUSE proxy from UI failed ?\n");
					sr_ecp_msg->message(SYSTEM_ERROR, input.e, "MP:Creceive STOP pulse from UI failed");
					throw MP_main_error (SYSTEM_ERROR, (uint64_t) 0);
				} else {
					wyjscie = true;
					continue;
				}
			} else if (mp_state == MP_STATE_PAUSED) {
				perror("Creceive RESUME proxy from UI failed ?\n");
				sr_ecp_msg->message(SYSTEM_ERROR, input.e, "MP: receive RESUME pulse from UI failed");
				throw MP_main_error (SYSTEM_ERROR, (uint64_t) 0);
			}
		} else if (rcvid == 0) {
			if (ui_new_pulse) {
				ui_new_pulse = false;
				if (ui_pulse_code == MP_STOP) {
					terminate_all (_robot_m);
					return true;
				}

				if (ui_pulse_code == MP_PAUSE) {
					// printf("ui_new_pulse MP_PAUSED\n");
					mp_state = MP_STATE_PAUSED;
				}

				if (mp_state == MP_STATE_PAUSED) {// oczekujemy na resume
					if (ui_pulse_code == MP_RESUME) { // odebrano resume
						mp_state = MP_STATE_RUNNING;
						wyjscie = true;
					}
				} else {
					if (ui_pulse_code == MP_TRIGGER) { // odebrano trigger
						wyjscie = true;
						*trigger = true;
					}
				}
			}
			continue;

		}	 else if (rcvid > 0) {
			printf("MP_TRIGGER server receive strange message of type: \n");
		}
	}

	return false;

};






void mp_task::mp_initialize_communication() {

	uint64_t e;     // kod bledu systemowego
	short tmp;

	char* sr_net_attach_point = config->return_attach_point_name(configurator::CONFIG_SERVER, "sr_attach_point", "[ui]");
	char* ui_net_attach_point = config->return_attach_point_name(configurator::CONFIG_SERVER, "ui_attach_point", "[ui]");
	char* mp_attach_point =	config->return_attach_point_name(configurator::CONFIG_SERVER, "mp_attach_point");
	char* mp_pulse_attach_point = config->return_attach_point_name(configurator::CONFIG_SERVER, "mp_pulse_attach_point");

	mrrocpp_network_path = config->return_mrrocpp_network_path();

	// 	printf("sr_net_attach_point: %s\n",sr_net_attach_point);
	// byc moze do poprawki
	if (( sr_ecp_msg = new sr_ecp(MP, mp_attach_point, sr_net_attach_point)) == NULL) { // Obiekt do komuniacji z SR
		e = errno;
		perror ( "Unable to locate aSR\n");
		throw MP_main_error(SYSTEM_ERROR, (uint64_t) 0);
	}

	tmp = 0;
	// kilka sekund  (~1) na otworzenie urzadzenia
	while ((UI_fd = name_open(ui_net_attach_point, NAME_FLAG_ATTACH_GLOBAL))  < 0)
		if ((tmp++) < CONNECT_RETRY)
			usleep(1000 * CONNECT_DELAY);
		else {
			e = errno;
			perror("Connect to UI afailed\n");
			sr_ecp_msg->message (SYSTEM_ERROR, e, "MP: Connect to UI failed");		// TO NIE DZIALA
			throw MP_main_error(SYSTEM_ERROR, (uint64_t) 0);
		};

	// Rejestracja procesu MP
	// if ( (id_MP_name = qnx_name_attach(0L, argv[1])) == -1) {
	if ((mp_attach = name_attach(NULL, mp_attach_point, NAME_FLAG_ATTACH_GLOBAL)) == NULL) {
		e = errno;
		perror("Failed to attach Master Process\n");
		sr_ecp_msg->message (SYSTEM_ERROR, e, "MP: Failed to name attach");
		throw MP_main_error(SYSTEM_ERROR, (uint64_t) 0);
	}

	// Rejestracja kanalu dla pulsow start z procesu UI i ECP
	if (( mp_trigger_attach = name_attach(NULL, mp_pulse_attach_point, NAME_FLAG_ATTACH_GLOBAL)) == NULL) {
		e = errno;
		perror("Failed to attach UI Pulse chanel for Master Process\n");
		sr_ecp_msg->message (SYSTEM_ERROR, e, "MP: Failed to name attach  UI Pulse");
		throw MP_main_error(SYSTEM_ERROR, (uint64_t) 0);
	}

	mp_wait_for_ui_name_open();

	delete [] mp_attach_point;
	delete [] sr_net_attach_point;
	delete [] ui_net_attach_point;
	delete [] mp_pulse_attach_point;

};
// -------------------------------------------------------------------




// ---------------------------------------------------------------
bool mp_task::Move ( mp_generator& the_generator ) {
// Funkcja zwraca false gdy samoistny koniec ruchu
// Funkcja zwraca true gdy koniec ruchu wywolany jest przez STOP

// czyszczenie aby nie czekac na pulsy z ECP
	for (map <ROBOT_ENUM, mp_robot*>::iterator robot_m_iterator = robot_m.begin();
	        robot_m_iterator != robot_m.end(); robot_m_iterator++) {
		if (robot_m_iterator->second->new_pulse) {
			robot_m_iterator->second->robot_new_pulse_checked = false;
		}
	}

	//	if (debug_tmp)  	printf("w mp move start\n");
// by Y - linia ponizej dodana 26.02.2007 - usunac komentarz jak bedzie dzialalo
// ze wzgledu na obluge pulsow z UI w szczegolnosci stopu i wstrzymania
	if (mp_receive_ui_or_ecp_pulse (robot_m, the_generator)) return true;
	//			if (debug_tmp)  	printf("w mp move za mp_receive_ui_or_ecp_pulse\n");

// czyszczenie aby nie czekac na pulsy z ECP
	for (map <ROBOT_ENUM, mp_robot*>::iterator robot_m_iterator = robot_m.begin();
	        robot_m_iterator != robot_m.end(); robot_m_iterator++) {
		if (robot_m_iterator->second->new_pulse) {
			robot_m_iterator->second->robot_new_pulse_checked = false;
		}
	}

// (Inicjacja) generacja pierwszego kroku ruchu
	if (!the_generator.first_step() ) return false;

// printf("w mp move przed do\n");
	do { // realizacja ruchu

		// ew. odbior pulsu
		//		if (debug_tmp)  	printf("w mp move przed mp_receive_ui_pulse\n");
		// 	if (trigger) printf("Y\n"); else printf("N\n");
		// zadanie przygotowania danych od czujnikow
		all_sensors_initiate_reading(the_generator.sensor_m);
		//			if (debug_tmp) printf("w mp move przed execute_all\n");
		// wykonanie kroku ruchu przez wybrane roboty (zmienna communicate)
		execute_all (the_generator.robot_m);
		//			if (debug_tmp) printf("w mp move przed all_sensors_get_reading\n");
		// odczytanie danych z wszystkich czujnikow
		all_sensors_get_reading (the_generator.sensor_m);
		//		if (debug_tmp) printf("w mp move przed mp_receive_ui_or_ecp_pulse\n");
		// oczekiwanie na puls z ECP lub UI
		if (mp_receive_ui_or_ecp_pulse (robot_m, the_generator)) return true;
		//	if (mp_receive_ui_pulse (robot_m, &trigger)) return true;
		//			if (debug_tmp) printf("mp move za mp_receive_ui_pulse\n");

	} while ( the_generator.next_step() );
// end: do

// Porzadki koncowe
	return false;
}
; // end: Move()
// ------------------------------------------------------------------------



// ------------------------------------------------------------------------
bool mp_task::clear_gen_list () {
	if (!gen_list.empty()) {
		gen_list.clear();
	}

	return true;
}
// ------------------------------------------------------------------------

// ------------------------------------------------------------------------
bool mp_task::add_gen (mp_generator* gen_l) {
	if (!gen_list.empty()) {
		gen_list.push_back(gen_l);
	} else {
		gen_list.push_back(gen_l);
		gen_list_iterator = gen_list.begin();
	}

	return true;
}
// ------------------------------------------------------------------------



// ------------------------------------------------------------------------
#if 0		// not used any more (ptrojane)
bool mp_task::rm_gen (mp_generator* gen_l) {
	if (!gen_list.empty()) {
		std::list<mp_generator*>::iterator gen_local_list_iterator = find(gen_list.begin(), gen_list.end(), gen_l);
		if (gen_local_list_iterator != gen_list.end()) {
			gen_list.erase(gen_local_list_iterator);
			if (!gen_list.empty()) {
				gen_local_list_iterator = gen_list.begin();
			}
			return true;
		}
	}

	return false;
}
#endif
// ------------------------------------------------------------------------








// ------------------------------------------------------------------------
bool mp_task::scheduller_run () {

	short trigger;

	// mp_receive_ecp_pulse_return_td ret;
	int ret;
	mp_receive_pulse_struct_tdef input;

	if (gen_list.empty()) {
		printf ("gen_list empty w scheduller_run\n");
		return true;
	}

	while (1) {

		switch ((*gen_list_iterator)->phase) {
			case mp_generator::BEFORE_FIRST_STEP:
				// cout << "BEFORE_FIRST_STEP" << endl;
				if (!((*gen_list_iterator)->first_step () )) {
//					cout << "BEFORE_FIRST_STEP 1" << endl;
					(*gen_list_iterator)->phase = mp_generator::GS_FINISHED;
					return false;
				}
				//		cout << "BEFORE_FIRST_STEP 2" << endl;
				(*gen_list_iterator)->phase = mp_generator::AFTER_STEP;
//				cout << "BEFORE_FIRST_STEP 3" << endl;
				break;
			case mp_generator::AFTER_STEP:
				// cout << "AFTER_STEP" << endl;
				trigger = false;
				// ew. odbior pulsu
				if (mp_receive_ui_pulse (robot_m, &trigger) == true) return true;
//				cout << "AFTER_STEP 2" << endl;
				if (trigger) {
					for (list<mp_generator*>::iterator gen_local_list_iterator = gen_list.begin();
					        gen_local_list_iterator != gen_list.end(); gen_local_list_iterator++) {
						(*gen_local_list_iterator)->trigger = trigger;
					}
				}
				all_sensors_initiate_reading ((*gen_list_iterator)->sensor_m);
				(*gen_list_iterator)->phase = mp_generator::AFTER_INITIATE_READING;
				break;
			case mp_generator::AFTER_INITIATE_READING:
				// cout << "AFTER_INITIATE_READING" << endl;
				// odebranie wszystkich dostarczonych juz pulsow
				/*
				do 
				{
					ret = mp_receive_ecp_pulse (WITH_TIMEOUT);
				//					if (ret.rcvid == 0) cout << "AFTER_INITIATE_READING rcvid 0" << endl;
				} while (ret.rcvid == 0);
				*/
				ret = check_and_optional_wait_for_new_pulse (&input, NEW_UI_OR_ECP_PULSE, WITH_TIMEOUT);
				// sprawdzamy czy wszystkie ECP sa gotowe do komunikacji
				for (map <ROBOT_ENUM, mp_robot*>::iterator robot_m_iterator = (*gen_list_iterator)->robot_m.begin();
				        robot_m_iterator != (*gen_list_iterator)->robot_m.end(); robot_m_iterator++) {
					//jesli ktorykolwiek robot nie jest gotowy
					if ((robot_m_iterator->second->communicate) && (!((robot_m_iterator->second->new_pulse)
					        && ((robot_m_iterator->second->pulse_code == ECP_WAIT_FOR_COMMAND) ||
					            (robot_m_iterator->second->pulse_code == ECP_WAIT_FOR_NEXT_STATE))))) {
						(*gen_list_iterator)->phase = mp_generator::WAITING_FOR_ECP_BEFORE_EXECUTE_MOTION;

						// sprawdzenie czy wszystkie pozostale gen_set'y tez sa w tym stanie
						all_gen_sets_waiting_for_ECP_pulse = true;
						for (list<mp_generator*>::iterator gen_local_list_iterator = gen_list.begin();
						        gen_local_list_iterator != gen_list.end(); gen_local_list_iterator++) {
							// jesli ktorykolwiek generator nie jest jeszcze zawieszony
							if ((*gen_local_list_iterator)->phase != mp_generator::WAITING_FOR_ECP_BEFORE_EXECUTE_MOTION) {
								all_gen_sets_waiting_for_ECP_pulse = false;
								break;
							}
						}
						break;
					}
				} // end: for

				// jesli wszytkie aktywne roboty dla biezacego generatore sa gotowe
				if ((*gen_list_iterator)->phase != mp_generator::WAITING_FOR_ECP_BEFORE_EXECUTE_MOTION) {
					execute_all((*gen_list_iterator)->robot_m);
					(*gen_list_iterator)->phase = mp_generator::AFTER_EXECUTE_MOTION;
				}
				(*gen_list_iterator)->new_pulse_checked = false;

				break;
			case mp_generator::WAITING_FOR_ECP_BEFORE_EXECUTE_MOTION:
				// cout << "WAITING_FOR_ECP_BEFORE_EXECUTE_MOTION" << endl;
				// jesli wszystkie gen_set'y czekaja na puls z ECP
				if (all_gen_sets_waiting_for_ECP_pulse) {
					//			cout << "WAITING_FOR_ECP_BEFORE_EXECUTE_MOTION all_gen_sets_waiting_for_ECP_pulse" << endl;
					if ((*gen_list_iterator)->new_pulse_checked) {
						// odbieramy puls z ECP z zawieszaniem bo nie mamy zadnego pulsu, ktory by oczekiwal
						// cout << "WAITING_FOR_ECP_BEFORE_EXECUTE_MOTION all_gen_sets_waiting_for_ECP_pulse new_pulse_checked" << endl;
						// ret = mp_receive_ecp_pulse (WITHOUT_TIMEOUT);
						ret = check_and_optional_wait_for_new_pulse (&input, NEW_ECP_PULSE, WITHOUT_TIMEOUT);

						for (map <ROBOT_ENUM, mp_robot*>::iterator robot_m_iterator = robot_m.begin();
						        robot_m_iterator != robot_m.end(); robot_m_iterator++) {
							if ((robot_m_iterator->second->communicate) && ((robot_m_iterator->second->new_pulse)
							        && ((robot_m_iterator->second->pulse_code == ECP_WAIT_FOR_COMMAND) ||
							            (robot_m_iterator->second->pulse_code == ECP_WAIT_FOR_NEXT_STATE)))) {
								robot_m_iterator->second->robot_new_pulse_checked = true;
							}
						} // end: for

						// oznaczamy ze pozostale generatory nie zbadaly jeszcze biezacego pulsu od ECP
						for (list<mp_generator*>::iterator gen_local_list_iterator = gen_list.begin();
						        gen_local_list_iterator != gen_list.end(); gen_local_list_iterator++) {
							(*gen_local_list_iterator)->new_pulse_checked = false;
						}

					}
//					cout << "WAITING_FOR_ECP_BEFORE_EXECUTE_MOTION all_gen_sets_waiting_for_ECP_pulse new_pulse_checked za" << endl;
					(*gen_list_iterator)->new_pulse_checked = true;
				} else {
					// odebranie wszystkich dostarczonych juz pulsow

					ret = check_and_optional_wait_for_new_pulse (&input, NEW_UI_OR_ECP_PULSE, WITH_TIMEOUT);
				}
				//		cout << "WAITING_FOR_ECP_BEFORE_EXECUTE_MOTION za" << endl;
				(*gen_list_iterator)->phase = mp_generator::AFTER_EXECUTE_MOTION;
				// sprawdzamy czy wszystkie ECP sa gotowe do komunkacji
				for (map <ROBOT_ENUM, mp_robot*>::iterator robot_m_iterator = (*gen_list_iterator)->robot_m.begin();
				        robot_m_iterator != (*gen_list_iterator)->robot_m.end(); robot_m_iterator++) {
					if ((robot_m_iterator->second->communicate) && (!((robot_m_iterator->second->new_pulse)
					        && ((robot_m_iterator->second->pulse_code == ECP_WAIT_FOR_COMMAND) ||
					            (robot_m_iterator->second->pulse_code == ECP_WAIT_FOR_NEXT_STATE))))) {
						(*gen_list_iterator)->phase = mp_generator::WAITING_FOR_ECP_BEFORE_EXECUTE_MOTION;
						break;
					}
				} // end: for

				// jesli wszytkie aktywne roboty dla biezacego generatore sa gotowe
				if ((*gen_list_iterator)->phase != mp_generator::WAITING_FOR_ECP_BEFORE_EXECUTE_MOTION) {
					execute_all((*gen_list_iterator)->robot_m);
					(*gen_list_iterator)->phase = mp_generator::AFTER_EXECUTE_MOTION;
					all_gen_sets_waiting_for_ECP_pulse = false;
				}

				break;
			case mp_generator::AFTER_EXECUTE_MOTION:
				// cout << "AFTER_EXECUTE_MOTION" << endl;
				all_sensors_get_reading((*gen_list_iterator)->sensor_m);
				(*gen_list_iterator)->phase = mp_generator::AFTER_GET_READING;
				break;
			case mp_generator::AFTER_GET_READING:
				// cout << "AFTER_GET_READING" << endl;
				if (!((*gen_list_iterator)->next_step ())) {
					(*gen_list_iterator)->phase = mp_generator::GS_FINISHED;
					return false;
				}
				(*gen_list_iterator)->phase = mp_generator::AFTER_STEP;
				break;
			case mp_generator::GS_FINISHED:
				// 	cout << "GS_FINISHED" << endl;
				//	printf ("GS_FINISHED w scheduller_run\n");
				trigger = false;
				// ew. odbior pulsu
				if (mp_receive_ui_pulse (robot_m, &trigger)) return true;
//				cout << "AFTER_STEP 2" << endl;
				if (trigger) {
					for (list<mp_generator*>::iterator gen_local_list_iterator = gen_list.begin();
					        gen_local_list_iterator != gen_list.end(); gen_local_list_iterator++) {
						(*gen_local_list_iterator)->trigger = trigger;
					}
				}
				break;
			default:

				break;
		}

		// aktualizacja wskaznika na biezacy generator_set
		if (!((++gen_list_iterator) != gen_list.end()))
			gen_list_iterator =  gen_list.begin();

	}

	return true;

}
// ------------------------------------------------------------------------

void mp_task::wait_for_start () {
	// Oczekiwanie na zlecenie START od UI

	bool wyjscie = false;
	mp_receive_pulse_struct_tdef input;

	while (!wyjscie) {
		check_and_optional_wait_for_new_pulse (&input, NEW_UI_PULSE, WITHOUT_TIMEOUT);
		if (ui_new_pulse) {
			ui_new_pulse = false;
			if (ui_pulse_code == MP_START) {
				wyjscie = true;
				continue;
			}
		}
	}

	sr_ecp_msg->message("MP user program is running");
}// end: wait_for_start ()
// ------------------------------------------------------------------------


void mp_task::wait_for_stop (WAIT_FOR_STOP_ENUM tryb) {// by Y zmodyfikowane w celu skrocenia kodu mp_m
	// Oczekiwanie na zlecenie STOP od UI

	sr_ecp_msg->message("To terminate user program click STOP icon");

	int rcvid;
	bool wyjscie = false;
	mp_receive_pulse_struct_tdef input;


	while (!wyjscie) {
		rcvid = check_and_optional_wait_for_new_pulse (&input, NEW_UI_PULSE, WITHOUT_TIMEOUT);
		if (rcvid == -1)/* Error condition, exit */
		{
			if (input.e != ETIMEDOUT) {
				perror("Receive StopProxy failed (MP)\n");
				sr_ecp_msg->message(SYSTEM_ERROR, input.e, "MP: Receive StopProxy failed");
				if (tryb == MP_THROW)	 throw MP_main_error(SYSTEM_ERROR, (uint64_t) 0);
				else if (tryb == MP_EXIT) exit(EXIT_FAILURE);
				else printf("bledny tryb w wait_for_stop\n");
			}
		} else {
			// if UI pulse occured
			if (ui_new_pulse) {
				ui_new_pulse = false;
				if (ui_pulse_code == MP_STOP) {
					wyjscie = true;
					continue;
				}
			}
		}

	}
}
; // end: wait_for_stop()
// ------------------------------------------------------------------------



// ------------------------------------------------------------------------

void mp_task::start_all (map <ROBOT_ENUM, mp_robot*>& _robot_m) {
// Wystartowanie wszystkich ECP

	int ret;
	map <ROBOT_ENUM, mp_robot*>::iterator robot_m_iterator;
	mp_receive_pulse_struct_tdef input;

	map <ROBOT_ENUM, mp_robot*> robots_m_tmp, robots_m_tmp2;


	// printf("start_all start\n");
	// przepisanie mapy robotow do skomunikowania na wersje tymczasowa
	robots_m_tmp = _robot_m;

	while (!(robots_m_tmp.empty())) {

		// przygotowanie wersji tymczasowej do usuwania robotow
		robots_m_tmp2.clear();

		// printf("start_all iter\n");
		for (robot_m_iterator = robots_m_tmp.begin(); robot_m_iterator != robots_m_tmp.end(); robot_m_iterator++) {
			if (robot_m_iterator->second->new_pulse ) {
				if (robot_m_iterator->second->pulse_code == ECP_WAIT_FOR_START) {
					robot_m_iterator->second->new_pulse = false;
					robot_m_iterator->second->robot_new_pulse_checked = false;
					robot_m_iterator->second->start_ecp();
				} else {
					printf("phase 2 bledny kod pulsu w start_all\n");
					throw MP_main_error(NON_FATAL_ERROR, INVALID_ECP_PULSE_IN_MP_START_ALL);
				}
			} else {
				// dodaj robota do listy jeszcze nie obsluzonych
				robots_m_tmp2[robot_m_iterator->first] = robot_m_iterator->second;
			}
		} // end: for

		// ponowne przepisanie map
		//robots_m_tmp.clear();
		robots_m_tmp = robots_m_tmp2;

		if (!(robots_m_tmp.empty())) {
			ret = check_and_optional_wait_for_new_pulse (&input, NEW_ECP_PULSE, WITHOUT_TIMEOUT);
		}

	} // end while (!(robots_m_tmp.empty()))

	// printf("start_all end\n");

} // end: mp_task::start_all()
// ------------------------------------------------------------------------


// ------------------------------------------------------------------------
void mp_task::execute_all (map <ROBOT_ENUM, mp_robot*>& _robot_m) {
// Wystartowanie wszystkich ECP

	int ret;
	map <ROBOT_ENUM, mp_robot*>::iterator robot_m_iterator;
	mp_receive_pulse_struct_tdef input;

	map <ROBOT_ENUM, mp_robot*> robots_m_tmp, robots_m_tmp2;

	// przepisanie mapy robotow do skomunikowania na wersje tymczasowa
	for (robot_m_iterator = _robot_m.begin(); robot_m_iterator != _robot_m.end(); robot_m_iterator++) {
		if (robot_m_iterator->second->communicate) {
			robots_m_tmp[robot_m_iterator->first] = robot_m_iterator->second;
		}
	}

	while (!(robots_m_tmp.empty())) {

		// przygotowanie wersji tymczasowej do usuwania robotow
		robots_m_tmp2.clear();

		for (robot_m_iterator = robots_m_tmp.begin(); robot_m_iterator != robots_m_tmp.end(); robot_m_iterator++) {
			// komunikujemy sie tylko z aktywnymi robotami
			if (robot_m_iterator->second->new_pulse) {
				if ((robot_m_iterator->second->pulse_code == ECP_WAIT_FOR_COMMAND) ||
				        (robot_m_iterator->second->pulse_code == ECP_WAIT_FOR_NEXT_STATE)) {
					robot_m_iterator->second->new_pulse = false;
					robot_m_iterator->second->robot_new_pulse_checked = false;
					robot_m_iterator->second->execute_motion();
				} else {
					printf("phase 2 bledny kod pulsu w execute_all, %d\n", robot_m_iterator->second->pulse_code);
					throw MP_main_error(NON_FATAL_ERROR, INVALID_ECP_PULSE_IN_MP_EXECUTE_ALL);
				}
			} else {
				// dodaj robota do listy jeszcze nie obsluzonych
				robots_m_tmp2[robot_m_iterator->first] = robot_m_iterator->second;
			}
		} // end: for

		// ponowne przepisanie map
		// robots_m_tmp.clear();
		robots_m_tmp = robots_m_tmp2;

		if (!(robots_m_tmp.empty())) {
			ret = check_and_optional_wait_for_new_pulse (&input, NEW_ECP_PULSE, WITHOUT_TIMEOUT);
		}

	} // end while (!(robots_m_tmp.empty()))

} // end: mp_task::execute_all()
// ------------------------------------------------------------------------


// ------------------------------------------------------------------------
void mp_task::terminate_all (map <ROBOT_ENUM, mp_robot*>& _robot_m) {
// Zatrzymanie wszystkich ECP
	int ret;
	map <ROBOT_ENUM, mp_robot*>::iterator robot_m_iterator;
	mp_receive_pulse_struct_tdef input;

	map <ROBOT_ENUM, mp_robot*> robots_m_tmp, robots_m_tmp2;

	// przepisanie mapy robotow do skomunikowania na wersje tymczasowa
	robots_m_tmp = _robot_m;

	while (!(robots_m_tmp.empty())) {
		//	ret = check_and_optional_wait_for_new_pulse (&input, NEW_ECP_PULSE, WITHOUT_TIMEOUT);

		// przygotowanie wersji tymczasowej do usuwania robotow
		robots_m_tmp2.clear();

		for (robot_m_iterator = robots_m_tmp.begin(); robot_m_iterator != robots_m_tmp.end(); robot_m_iterator++) {

			if (robot_m_iterator->second->new_pulse) {
//				if ((robot_m_iterator->second->pulse_code == ECP_WAIT_FOR_STOP) || (robot_m_iterator->second->pulse_code == ECP_WAIT_FOR_COMMAND))
				if (1) {

					robot_m_iterator->second->new_pulse = false;
					robot_m_iterator->second->robot_new_pulse_checked = false;
					robot_m_iterator->second->terminate_ecp();

				} else {

					printf("phase 2 bledny kod pulsu w terminate_all\n");
					throw MP_main_error(NON_FATAL_ERROR, INVALID_ECP_PULSE_IN_MP_TERMINATE_ALL);
				}
			} else {
				// dodaj robota do listy jeszcze nie obsluzonych
				robots_m_tmp2[robot_m_iterator->first] = robot_m_iterator->second;
			}
		} // end: for
		// ponowne przepisanie map
		// robots_m_tmp.clear();
		robots_m_tmp = robots_m_tmp2;

		if (!(robots_m_tmp.empty())) {
			ret = check_and_optional_wait_for_new_pulse (&input, NEW_ECP_PULSE, WITHOUT_TIMEOUT);
		}

	} // end while (!(robots_m_tmp.empty()))

} // end: mp_task::terminate_all()
// ------------------------------------------------------------------------


// ------------------------------------------------------------------------
void mp_task::kill_all_ECP (map <ROBOT_ENUM, mp_robot*>& _robot_m) {

	// Zabicie wszystkich ECP z mapy

	for (map <ROBOT_ENUM, mp_robot*>::iterator robot_m_iterator = _robot_m.begin();
	        robot_m_iterator != _robot_m.end(); robot_m_iterator++) {
		SignalKill(robot_m_iterator->second->nd, robot_m_iterator->second->ECP_pid, 0, SIGTERM, 0, 0);
	}

} // end: kill_all_ECP()
// ------------------------------------------------------------------------
