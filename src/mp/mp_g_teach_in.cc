// -------------------------------------------------------------------------
//                              mp.cc
//
// MP Master Process - methods
//
// -------------------------------------------------------------------------
// Funkcje do konstruowania procesow MP

#include <stdio.h>
#include <signal.h>
#include <stdarg.h>
#include <errno.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <fstream>

#include <boost/foreach.hpp>

#if defined(__QNXNTO__)
#include <sys/iofunc.h>
#include <sys/dispatch.h>
#include <sys/neutrino.h>
#include <sys/netmgr.h>
#endif

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/srlib.h"
#include "mp/mp.h"
#include "mp/mp_g_teach_in.h"

namespace mrrocpp {
namespace mp {
namespace generator {


using namespace std;




teach_in::teach_in(task::task& _mp_task)
	: generator (_mp_task), UI_fd(_mp_task.UI_fd)
{
	pose_list.clear();
	pose_list_iterator = pose_list.end();
}

// -------------------------------------------------------
// destruktor
teach_in::~teach_in (void) { flush_pose_list(); }




// --------------------------------------------------------------------------
// Zapis trajektorii do pliku
void teach_in::save_file (lib::POSE_SPECIFICATION ps) {
	lib::ECP_message ecp_to_ui_msg; // Przesylka z ECP do UI
	lib::UI_reply ui_to_ecp_rep;    // Odpowiedz UI do ECP
	common::mp_taught_in_pose tip;        // Zapisywana pozycja
	char *cwd;                 // Wsk. na nazwe biezacego katalogu
	char coordinate_type[80];  // Opis wspolrzednych: "MOTOR", "JOINT", ...
	uint64_t e;       // Kod bledu systemowego
	uint64_t number_of_poses; // Liczba pozycji do zapamietania
	uint64_t i, j;    // Liczniki petli

	ecp_to_ui_msg.hdr.type = 0;

	ecp_to_ui_msg.ecp_message = lib::SAVE_FILE;   // Polecenie wprowadzenia nazwy pliku
	strcpy(ecp_to_ui_msg.string, "*.trj");   // Wzorzec nazwy pliku
// if ( Send (UI_pid, &ecp_to_ui_msg, &ui_to_ecp_rep, sizeof(lib::ECP_message), sizeof(lib::UI_reply)) == -1) {
#if !defined(USE_MESSIP_SRR)
	if (MsgSend(UI_fd, &ecp_to_ui_msg,  sizeof(lib::ECP_message), &ui_to_ecp_rep, sizeof(lib::UI_reply)) < 0) {// by Y&W
#else
	int status;
	if(messip_send(UI_fd, 0, 0, &ecp_to_ui_msg, sizeof(lib::ECP_message),
					&status, &ui_to_ecp_rep, sizeof(lib::UI_reply), MESSIP_NOTIMEOUT) < 0) {
#endif
		e = errno;
		perror("ECP: Send() to UI failed");
		sr_ecp_msg.message (lib::SYSTEM_ERROR, e, "ECP: Send() to UI failed");
		throw generator::MP_error(lib::SYSTEM_ERROR, (uint64_t) 0);
	}
	if ( ui_to_ecp_rep.reply == lib::QUIT) // Nie wybrano nazwy pliku lub zrezygnowano z zapisu
		return;
	switch (ps) {
		case lib::MOTOR:
			strcpy (coordinate_type, "MOTOR");
			break;
		case lib::JOINT:
			strcpy (coordinate_type, "JOINT");
			break;
		case lib::XYZ_ANGLE_AXIS:
			strcpy (coordinate_type, "XYZ_ANGLE_AXIS");
			break;
		case lib::XYZ_EULER_ZYZ:
			strcpy (coordinate_type, "XYZ_EULER_ZYZ");
			break;
		case lib::PF_VELOCITY:
			strcpy (coordinate_type, "lib::PF_VELOCITY");
			break;

		default:
			strcpy (coordinate_type, "MOTOR");
	}
	cwd = getcwd (NULL, 0);
	if ( chdir(ui_to_ecp_rep.path) != 0 ) {
		perror(ui_to_ecp_rep.path);
		throw generator::MP_error(lib::NON_FATAL_ERROR, NON_EXISTENT_DIRECTORY);
	}
	ofstream to_file(ui_to_ecp_rep.filename); // otworz plik do zapisu
	e = errno;
	if (!to_file) {
		perror(ui_to_ecp_rep.filename);
		throw generator::MP_error(lib::NON_FATAL_ERROR, NON_EXISTENT_FILE);
	} else {
		initiate_pose_list();
		number_of_poses = pose_list_length();
		to_file << coordinate_type << '\n';
		to_file << number_of_poses << '\n';
		for ( i = 0; i < number_of_poses; i++) {
			get_pose (tip);
			to_file << tip.motion_time << ' ';
			for (j = 0; j < MAX_SERVOS_NR; j++)
				to_file << tip.coordinates[j] << ' ';
			if (ps == lib::PF_VELOCITY) { // by Y
				to_file << tip.extra_info << ' ';
			}
			to_file << '\n';
			next_pose_list_ptr();
		}
		initiate_pose_list();
	}
} // end: mp_save_file()
// --------------------------------------------------------------------------



// --------------------------------------------------------------------------
// Wczytanie trajektorii z pliku
bool teach_in::load_file_with_path (const char* file_name, short robot_number) {
// Funkcja zwraca true jesli wczytanie trajektorii powiodlo sie,
// false w przeciwnym razie
// common::mp_taught_in_pose tip;        // Wczytana pozycja
// char *cwd;                 // Wsk. na nazwe biezacego katalogu
	char coordinate_type[80];  // Opis wspolrzednych: "MOTOR", "JOINT", ...
	lib::POSE_SPECIFICATION ps;     // Rodzaj wspolrzednych
	// uint64_t e;       // Kod bledu systemowego
	uint64_t number_of_poses; // Liczba zapamietanych pozycji
	uint64_t i, j;    // Liczniki petli
	bool first_time = true; // Znacznik
	double irp6ot_coordinates[MAX_SERVOS_NR];     // Wczytane wspolrzedne
	double irp6p_coordinates[MAX_SERVOS_NR];     // Wczytane wspolrzedne
	double motion_time;        // Czas dojscia do wspolrzednych


	ifstream from_file(file_name); // otworz plik do odczytu
	if (!from_file.good()) {
		perror(file_name);
		throw generator::MP_error(lib::NON_FATAL_ERROR, NON_EXISTENT_FILE);
	}

	if ( !(from_file >> coordinate_type) ) {
		throw generator::MP_error (lib::NON_FATAL_ERROR, READ_FILE_ERROR);
	}

// Usuwanie spacji i tabulacji
	i = 0;
	j = 0;
	while ( coordinate_type[i] == ' ' || coordinate_type[i] == '\t') i++;
	while ( coordinate_type[i] != ' '   && coordinate_type[i] != '\t' &&
	        coordinate_type[i] != '\n'  && coordinate_type[i] != '\r' &&
	        coordinate_type[j] != '\0' ) {
		coordinate_type[j] = toupper(coordinate_type[i]);
		i++;
		j++;
	}
	coordinate_type[j] = '\0';

	if ( !strcmp(coordinate_type, "MOTOR") ) {
		ps = lib::MOTOR;
	} else if ( !strcmp(coordinate_type, "JOINT") )
		ps = lib::JOINT;
	else if ( !strcmp(coordinate_type, "XYZ_ANGLE_AXIS") )
		ps = lib::XYZ_ANGLE_AXIS;
	else if ( !strcmp(coordinate_type, "XYZ_EULER_ZYZ") )
		ps = lib::XYZ_EULER_ZYZ;
	else if ( !strcmp(coordinate_type, "lib::PF_VELOCITY") )
		ps = lib::PF_VELOCITY;
	else {
		throw generator::MP_error(lib::NON_FATAL_ERROR, NON_TRAJECTORY_FILE);
	}

	if ( !(from_file >> number_of_poses) ) {
		throw generator::MP_error (lib::NON_FATAL_ERROR, READ_FILE_ERROR);
	}

	flush_pose_list(); // Usuniecie listy pozycji, o ile istnieje
	for ( i = 0; i < number_of_poses; i++) {

		if (!(from_file >> motion_time)) {
			throw generator::MP_error (lib::NON_FATAL_ERROR, READ_FILE_ERROR);
		}
		for ( j = 0; j < MAX_SERVOS_NR; j++) {
			if ( !(from_file >> irp6ot_coordinates[j]) ) { // Zabezpieczenie przed danymi nienumerycznymi
				throw generator::MP_error (lib::NON_FATAL_ERROR, READ_FILE_ERROR);
			}
		}
		if (robot_number > 1) {
			for ( j = 0; j < MAX_SERVOS_NR; j++) {
				if ( !(from_file >> irp6p_coordinates[j]) ) { // Zabezpieczenie przed danymi nienumerycznymi
					throw generator::MP_error (lib::NON_FATAL_ERROR, READ_FILE_ERROR);
				}
			}
		}

		if (first_time) {
			// Tworzymy glowe listy
			first_time = false;
			if (robot_number > 1) {
				create_pose_list_head(ps, motion_time, irp6ot_coordinates, irp6p_coordinates);
			} else {
				create_pose_list_head(ps, motion_time, irp6ot_coordinates);
			}
		} else {
			// Wstaw do listy nowa pozycje
			if (robot_number > 1) {
				insert_pose_list_element(ps, motion_time, irp6ot_coordinates, irp6p_coordinates);
			} else {
				insert_pose_list_element(ps, motion_time, irp6ot_coordinates);
			}
		}

	} // end: for

	return true;
} // end: load_file()
// --------------------------------------------------------------------------




// --------------------------------------------------------------------------
// Wczytanie trajektorii z pliku
bool teach_in::load_file () {
// Funkcja zwraca true jesli wczytanie trajektorii powiodlo sie,
// false w przeciwnym razie
	lib::ECP_message ecp_to_ui_msg; // Przesylka z ECP do UI
	lib::UI_reply ui_to_ecp_rep;    // Odpowiedz UI do ECP
// common::mp_taught_in_pose tip;        // Wczytana pozycja
// char *cwd;                 // Wsk. na nazwe biezacego katalogu
	char coordinate_type[80];  // Opis wspolrzednych: "MOTOR", "JOINT", ...
	lib::POSE_SPECIFICATION ps;     // Rodzaj wspolrzednych
	uint64_t e;       // Kod bledu systemowego
	uint64_t number_of_poses; // Liczba zapamietanych pozycji
	uint64_t i, j;    // Liczniki petli
	bool first_time = true; // Znacznik
	double coordinates[MAX_SERVOS_NR];     // Wczytane wspolrzedne
	int extra_info;				// by Y - dodatkowe info do dowolnego wykorzystania
	double motion_time;        // Czas dojscia do wspolrzednych

	ecp_to_ui_msg.hdr.type = 0;

	ecp_to_ui_msg.ecp_message = lib::LOAD_FILE;   // Polecenie wprowadzenia nazwy odczytywanego pliku

// if ( Send (UI_pid, &ecp_to_ui_msg, &ui_to_ecp_rep, sizeof(lib::ECP_message), sizeof(lib::UI_reply)) == -1) {
#if !defined(USE_MESSIP_SRR)
	if (MsgSend(UI_fd, &ecp_to_ui_msg, sizeof(lib::ECP_message), &ui_to_ecp_rep, sizeof(lib::UI_reply)) < 0) {// by Y&W
#else
	int status;
	if(messip_send(UI_fd, 0, 0, &ecp_to_ui_msg, sizeof(lib::ECP_message),
					&status, &ui_to_ecp_rep, sizeof(lib::UI_reply), MESSIP_NOTIMEOUT) < 0) {
#endif
		e = errno;
		perror("ECP: Send() to UI failed");
		sr_ecp_msg.message (lib::SYSTEM_ERROR, e, "ECP: Send() to UI failed");
		throw generator::MP_error(lib::SYSTEM_ERROR, (uint64_t) 0);
	}
	if ( ui_to_ecp_rep.reply == lib::QUIT) // Nie wybrano nazwy pliku lub zrezygnowano z zapisu
		return false;
	if ( chdir(ui_to_ecp_rep.path) != 0 ) {
		perror(ui_to_ecp_rep.path);
		throw generator::MP_error(lib::NON_FATAL_ERROR, NON_EXISTENT_DIRECTORY);
	}

	ifstream from_file(ui_to_ecp_rep.filename); // otworz plik do odczytu
	if (!from_file.good()) {
		perror(ui_to_ecp_rep.filename);
		throw generator::MP_error(lib::NON_FATAL_ERROR, NON_EXISTENT_FILE);
	}
	if ( !(from_file >> coordinate_type) ) {
		throw generator::MP_error (lib::NON_FATAL_ERROR, READ_FILE_ERROR);
	}

// Usuwanie spacji i tabulacji
	i = 0;
	j = 0;
	while ( coordinate_type[i] == ' ' || coordinate_type[i] == '\t') i++;
	while ( coordinate_type[i] != ' '   && coordinate_type[i] != '\t' &&
	        coordinate_type[i] != '\n'  && coordinate_type[i] != '\r' &&
	        coordinate_type[j] != '\0' ) {
		coordinate_type[j] = toupper(coordinate_type[i]);
		i++;
		j++;
	}
	coordinate_type[j] = '\0';

	if ( !strcmp(coordinate_type, "MOTOR") ) {
		ps = lib::MOTOR;
	} else if ( !strcmp(coordinate_type, "JOINT") )
		ps = lib::JOINT;
	else if ( !strcmp(coordinate_type, "XYZ_ANGLE_AXIS") )
		ps = lib::XYZ_ANGLE_AXIS;
	else if ( !strcmp(coordinate_type, "XYZ_EULER_ZYZ") )
		ps = lib::XYZ_EULER_ZYZ;
	else if ( !strcmp(coordinate_type, "lib::PF_VELOCITY") )
		ps = lib::PF_VELOCITY;
	else {
		throw generator::MP_error(lib::NON_FATAL_ERROR, NON_TRAJECTORY_FILE);
	}
	if ( !(from_file >> number_of_poses) ) {
		throw generator::MP_error (lib::NON_FATAL_ERROR, READ_FILE_ERROR);
	}
	flush_pose_list(); // Usuniecie listy pozycji, o ile istnieje
	for ( i = 0; i < number_of_poses; i++) {
		if (!(from_file >> motion_time)) {
			throw generator::MP_error (lib::NON_FATAL_ERROR, READ_FILE_ERROR);
		}
		for ( j = 0; j < MAX_SERVOS_NR; j++) {
			if ( !(from_file >> coordinates[j]) ) { // Zabezpieczenie przed danymi nienumerycznymi
				throw generator::MP_error (lib::NON_FATAL_ERROR, READ_FILE_ERROR);
			}
		}

		if (ps == lib::PF_VELOCITY) { // by Y
			if ( !(from_file >> extra_info) ) { // Zabezpieczenie przed danymi nienumerycznymi
				throw generator::MP_error (lib::NON_FATAL_ERROR, READ_FILE_ERROR);
			}
			if (first_time) {
				// Tworzymy glowe listy
				first_time = false;
				create_pose_list_head(ps, motion_time, extra_info, coordinates);
			} else {
				// Wstaw do listy nowa pozycje
				insert_pose_list_element(ps, motion_time, extra_info, coordinates);
			}
		} else {
			if (first_time) {
				// Tworzymy glowe listy
				first_time = false;
				create_pose_list_head(ps, motion_time, coordinates);
			} else {
				// Wstaw do listy nowa pozycje
				insert_pose_list_element(ps, motion_time, coordinates);
			}
		}
	} // end: for

	return true;
} // end: load_file()
// --------------------------------------------------------------------------



// -------------------------------------------------------
void teach_in::flush_pose_list ( void ) {
	pose_list.clear();
}
// -------------------------------------------------------
void teach_in::initiate_pose_list(void) { pose_list_iterator = pose_list.begin();};
// -------------------------------------------------------
void teach_in::next_pose_list_ptr (void) {
	if (pose_list_iterator != pose_list.end())
		pose_list_iterator++;
}
// -------------------------------------------------------
void teach_in::get_pose (common::mp_taught_in_pose& tip) { // by Y
	tip.arm_type = pose_list_iterator->arm_type;
	tip.motion_time = pose_list_iterator->motion_time;
	tip.extra_info = pose_list_iterator->extra_info;
	memcpy(tip.coordinates, pose_list_iterator->coordinates, MAX_SERVOS_NR*sizeof(double));
	memcpy(tip.irp6p_coordinates, pose_list_iterator->irp6p_coordinates, MAX_SERVOS_NR*sizeof(double));
}
// -------------------------------------------------------
// Pobierz nastepna pozycje z listy
void teach_in::get_next_pose (double next_pose[MAX_SERVOS_NR])  {
	memcpy(next_pose, pose_list_iterator->coordinates, MAX_SERVOS_NR*sizeof(double));
}

void teach_in::get_next_pose (double next_pose[MAX_SERVOS_NR], double irp6_next_pose[MAX_SERVOS_NR])  {
	memcpy(next_pose, pose_list_iterator->coordinates, MAX_SERVOS_NR*sizeof(double));
	memcpy(irp6_next_pose, pose_list_iterator->irp6p_coordinates, MAX_SERVOS_NR*sizeof(double));
}


// -------------------------------------------------------
void teach_in::set_pose (lib::POSE_SPECIFICATION ps, double motion_time, double coordinates[MAX_SERVOS_NR]) {
	pose_list_iterator->arm_type = ps;
	pose_list_iterator->motion_time = motion_time;
	memcpy(pose_list_iterator->coordinates, coordinates, MAX_SERVOS_NR*sizeof(double));
}

void teach_in::set_pose (lib::POSE_SPECIFICATION ps, double motion_time, double coordinates[MAX_SERVOS_NR],
                                      double irp6p_coordinates[MAX_SERVOS_NR]) {
	pose_list_iterator->arm_type = ps;
	pose_list_iterator->motion_time = motion_time;
	memcpy(pose_list_iterator->coordinates, coordinates, MAX_SERVOS_NR*sizeof(double));
	memcpy(pose_list_iterator->irp6p_coordinates, irp6p_coordinates, MAX_SERVOS_NR*sizeof(double));
}

// -------------------------------------------------------
bool teach_in::is_pose_list_element ( void ) {
	// sprawdza czy aktualnie wskazywany jest element listy, czy lista sie skonczyla
	if ( pose_list_iterator != pose_list.end())
		return true;
	else
		return false;
}
// -------------------------------------------------------
bool teach_in::is_last_list_element ( void ) {
	// sprawdza czy aktualnie wskazywany element listy ma nastepnik
	// jesli <> nulla
	if ( pose_list_iterator != pose_list.end() ) {
		if ( (++pose_list_iterator) != pose_list.end() ) {
			--pose_list_iterator;
			return false;
		}  else {
			--pose_list_iterator;
			return true;
		}
		; // end if
	}
	return false;
}
// -------------------------------------------------------

void teach_in::create_pose_list_head (lib::POSE_SPECIFICATION ps, double motion_time, double coordinates[MAX_SERVOS_NR]) {
	pose_list.push_back(common::mp_taught_in_pose(ps, motion_time, coordinates));
	pose_list_iterator = pose_list.begin();
}

// by Y

void teach_in::create_pose_list_head (lib::POSE_SPECIFICATION ps, double motion_time, double coordinates[MAX_SERVOS_NR],
        double irp6p_coordinates[MAX_SERVOS_NR]) {
	pose_list.push_back(common::mp_taught_in_pose(ps, motion_time, coordinates, irp6p_coordinates));
	pose_list_iterator = pose_list.begin();
}


void teach_in::create_pose_list_head (lib::POSE_SPECIFICATION ps, double motion_time, int extra_info, double coordinates[MAX_SERVOS_NR]) {
	pose_list.push_back(common::mp_taught_in_pose(ps, motion_time, extra_info, coordinates));
	pose_list_iterator = pose_list.begin();
}

void teach_in::insert_pose_list_element (lib::POSE_SPECIFICATION ps, double motion_time, double coordinates[MAX_SERVOS_NR]) {
	pose_list.push_back(common::mp_taught_in_pose(ps, motion_time, coordinates));
	pose_list_iterator++;
}

// by Y

void teach_in::insert_pose_list_element (lib::POSE_SPECIFICATION ps, double motion_time, double coordinates[MAX_SERVOS_NR],
        double irp6p_coordinates[MAX_SERVOS_NR]) {
	pose_list.push_back(common::mp_taught_in_pose(ps, motion_time, coordinates, irp6p_coordinates));
	pose_list_iterator++;
}


void teach_in::insert_pose_list_element (lib::POSE_SPECIFICATION ps, double motion_time, int extra_info, double coordinates[MAX_SERVOS_NR]) {
	pose_list.push_back(common::mp_taught_in_pose(ps, motion_time, extra_info, coordinates));
	pose_list_iterator++;
}

// -------------------------------------------------------
int teach_in::pose_list_length(void) { return pose_list.size(); };
// -------------------------------------------------------



// ----------------------------------------------------------------------------------------------
// ----------------------  metoda	first_step -------------------------------------------------
// ----------------------------------------------------------------------------------------------

bool teach_in::first_step () {
//  printf("w teach_in::first_step\n");
	idle_step_counter = 1;

//   printf("w teach_in::first_step 2\n");
	initiate_pose_list();

	BOOST_FOREACH(const common::robot_pair_t & robot_node, robot_m) {
		robot_node.second->ecp_td.mp_command = lib::NEXT_POSE;
		robot_node.second->ecp_td.instruction_type = lib::GET;
		robot_node.second->ecp_td.get_type = ARM_DV;
		robot_node.second->ecp_td.get_arm_type = lib::MOTOR;
		robot_node.second->communicate = true;
	}

//  printf("w teach_in::first_step za initiate_pose_list\n");
	return next_step ();
}

// ----------------------------------------------------------------------------------------------
// ----------------------  metoda	next_step --------------------------------------------------
// ----------------------------------------------------------------------------------------------

bool teach_in::next_step () {
	common::mp_taught_in_pose tip;      // Nauczona pozycja

// printf("W teach_in::next_step\n");
	if ( idle_step_counter ) { // Oczekiwanie na odczyt aktualnego polozenia koncowki
		idle_step_counter--;
		return true;
	}

// printf("W teach_in::next_step\n");
	/*
	  if (is_pose_list_element ())
	    the_robot.set_ecp_reply (lib::ECP_ACKNOWLEDGE);
	  else
	    the_robot.set_ecp_reply (lib::TASK_TERMINATED);
	*/
// printf("W teach_in::next_step przed mp_buffer_receive_and_send\n");
// the_robot.mp_buffer_receive_and_send ();
// printf("W teach_in::next_step za mp_buffer_receive_and_send\n");

	if (!is_pose_list_element ())
		return false; // Jezeli lista jest pusta to konczymy generacje trajektorii

// printf("W teach_in::next_step\n");

	get_pose (tip);

	common::robots_t::iterator robot_m_iterator;

	// Przepisanie pozycji z listy

	robot_m_iterator = robot_m.find(lib::ROBOT_IRP6_ON_TRACK);
	if(robot_m_iterator != robot_m.end()) {
		switch ( tip.arm_type ) {
			case lib::C_MOTOR:
				robot_m_iterator->second->ecp_td.instruction_type = lib::SET;
				robot_m_iterator->second->ecp_td.set_type = ARM_DV; // ARM
				robot_m_iterator->second->ecp_td.set_arm_type = lib::MOTOR;
				robot_m_iterator->second->ecp_td.motion_type = lib::ABSOLUTE;
				robot_m_iterator->second->ecp_td.next_interpolation_type = lib::MIM;
				robot_m_iterator->second->ecp_td.motion_steps = (uint16_t) ceil(tip.motion_time / STEP);
				robot_m_iterator->second->ecp_td.value_in_step_no = robot_m_iterator->second->ecp_td.motion_steps;
				memcpy (robot_m_iterator->second->ecp_td.next_motor_arm_coordinates, tip.coordinates, MAX_SERVOS_NR*sizeof (double));
				break;
			case lib::C_JOINT:
				robot_m_iterator->second->ecp_td.instruction_type = lib::SET;
				robot_m_iterator->second->ecp_td.set_type = ARM_DV; // ARM
				robot_m_iterator->second->ecp_td.set_arm_type = lib::JOINT;
				robot_m_iterator->second->ecp_td.motion_type = lib::ABSOLUTE;
				robot_m_iterator->second->ecp_td.next_interpolation_type = lib::MIM;
				robot_m_iterator->second->ecp_td.motion_steps = (uint16_t) ceil(tip.motion_time / STEP);
				robot_m_iterator->second->ecp_td.value_in_step_no = robot_m_iterator->second->ecp_td.motion_steps;
				memcpy (robot_m_iterator->second->ecp_td.next_joint_arm_coordinates, tip.coordinates, MAX_SERVOS_NR*sizeof (double));
				break;
			case lib::C_XYZ_EULER_ZYZ:
				robot_m_iterator->second->ecp_td.instruction_type = lib::SET;
				robot_m_iterator->second->ecp_td.set_type = ARM_DV; // ARM
				robot_m_iterator->second->ecp_td.set_arm_type = lib::XYZ_EULER_ZYZ;
				robot_m_iterator->second->ecp_td.motion_type = lib::ABSOLUTE;
				robot_m_iterator->second->ecp_td.next_interpolation_type = lib::MIM;
				robot_m_iterator->second->ecp_td.motion_steps = (uint16_t) ceil(tip.motion_time / STEP);
				robot_m_iterator->second->ecp_td.value_in_step_no = robot_m_iterator->second->ecp_td.motion_steps;
				memcpy (robot_m_iterator->second->ecp_td.next_XYZ_ZYZ_arm_coordinates, tip.coordinates, 6*sizeof (double));
				robot_m_iterator->second->ecp_td.next_gripper_coordinate = tip.coordinates[6];
				break;
			case lib::C_XYZ_ANGLE_AXIS:
				robot_m_iterator->second->ecp_td.instruction_type = lib::SET;
				robot_m_iterator->second->ecp_td.set_type = ARM_DV; // ARM
				robot_m_iterator->second->ecp_td.set_arm_type = lib::XYZ_ANGLE_AXIS;
				robot_m_iterator->second->ecp_td.motion_type = lib::ABSOLUTE;
				robot_m_iterator->second->ecp_td.next_interpolation_type = lib::MIM;
				robot_m_iterator->second->ecp_td.motion_steps = (uint16_t) ceil(tip.motion_time / STEP);
				robot_m_iterator->second->ecp_td.value_in_step_no = robot_m_iterator->second->ecp_td.motion_steps;
				memcpy (robot_m_iterator->second->ecp_td.next_XYZ_AA_arm_coordinates, tip.coordinates, 6*sizeof (double));
				robot_m_iterator->second->ecp_td.next_gripper_coordinate = tip.coordinates[6];
				break;
			default:
				throw MP_error (lib::NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
		} // end: switch
	}

	robot_m_iterator = robot_m.find(lib::ROBOT_IRP6_POSTUMENT);
	if (robot_m_iterator != robot_m.end()) {
		//       	printf("postument\n");
		switch ( tip.arm_type ) {
			case lib::C_MOTOR:
				robot_m_iterator->second->ecp_td.instruction_type = lib::SET;
				robot_m_iterator->second->ecp_td.set_type = ARM_DV; // ARM
				robot_m_iterator->second->ecp_td.set_arm_type = lib::MOTOR;
				robot_m_iterator->second->ecp_td.motion_type = lib::ABSOLUTE;
				robot_m_iterator->second->ecp_td.next_interpolation_type = lib::MIM;
				robot_m_iterator->second->ecp_td.motion_steps = (uint16_t) ceil(tip.motion_time / STEP);
				robot_m_iterator->second->ecp_td.value_in_step_no = robot_m_iterator->second->ecp_td.motion_steps;
				memcpy (robot_m_iterator->second->ecp_td.next_motor_arm_coordinates, tip.irp6p_coordinates, MAX_SERVOS_NR*sizeof (double));
				break;
			case lib::C_JOINT:
				robot_m_iterator->second->ecp_td.instruction_type = lib::SET;
				robot_m_iterator->second->ecp_td.set_type = ARM_DV; // ARM
				robot_m_iterator->second->ecp_td.set_arm_type = lib::JOINT;
				robot_m_iterator->second->ecp_td.motion_type = lib::ABSOLUTE;
				robot_m_iterator->second->ecp_td.next_interpolation_type = lib::MIM;
				robot_m_iterator->second->ecp_td.motion_steps = (uint16_t) ceil(tip.motion_time / STEP);
				robot_m_iterator->second->ecp_td.value_in_step_no = robot_m_iterator->second->ecp_td.motion_steps;
				memcpy (robot_m_iterator->second->ecp_td.next_joint_arm_coordinates, tip.irp6p_coordinates, MAX_SERVOS_NR*sizeof (double));
				break;
			case lib::C_XYZ_EULER_ZYZ:
				robot_m_iterator->second->ecp_td.instruction_type = lib::SET;
				robot_m_iterator->second->ecp_td.set_type = ARM_DV; // ARM
				robot_m_iterator->second->ecp_td.set_arm_type = lib::XYZ_EULER_ZYZ;
				robot_m_iterator->second->ecp_td.motion_type = lib::ABSOLUTE;
				robot_m_iterator->second->ecp_td.next_interpolation_type = lib::MIM;
				robot_m_iterator->second->ecp_td.motion_steps = (uint16_t) ceil(tip.motion_time / STEP);
				robot_m_iterator->second->ecp_td.value_in_step_no = robot_m_iterator->second->ecp_td.motion_steps;
				memcpy (robot_m_iterator->second->ecp_td.next_XYZ_ZYZ_arm_coordinates, tip.irp6p_coordinates, 6*sizeof (double));
				robot_m_iterator->second->ecp_td.next_gripper_coordinate = tip.irp6p_coordinates[6];
				break;
			case lib::C_XYZ_ANGLE_AXIS:
				robot_m_iterator->second->ecp_td.instruction_type = lib::SET;
				robot_m_iterator->second->ecp_td.set_type = ARM_DV; // ARM
				robot_m_iterator->second->ecp_td.set_arm_type = lib::XYZ_ANGLE_AXIS;
				robot_m_iterator->second->ecp_td.motion_type = lib::ABSOLUTE;
				robot_m_iterator->second->ecp_td.next_interpolation_type = lib::MIM;
				robot_m_iterator->second->ecp_td.motion_steps = (uint16_t) ceil(tip.motion_time / STEP);
				robot_m_iterator->second->ecp_td.value_in_step_no = robot_m_iterator->second->ecp_td.motion_steps;
				memcpy (robot_m_iterator->second->ecp_td.next_XYZ_AA_arm_coordinates, tip.irp6p_coordinates, 6*sizeof (double));
				robot_m_iterator->second->ecp_td.next_gripper_coordinate = tip.irp6p_coordinates[6];
				break;
			default:
				throw MP_error (lib::NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
		} // end: switch
	}

	next_pose_list_ptr (); // nastepna pozycja
	return true;

}

} // namespace generator
} // namespace mp
} // namespace mrrocpp

