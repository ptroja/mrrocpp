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

#if defined(__QNXNTO__)
#include <sys/iofunc.h>
#include <sys/dispatch.h>
#include <sys/neutrino.h>
#include <sys/netmgr.h>
#endif

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"

#include "lib/srlib.h"
#include "mp/mp.h"
#include "mp/mp_g_teach_in.h"


using namespace std;




mp_teach_in_generator::mp_teach_in_generator(mp_task& _mp_task)
	: mp_generator (_mp_task), UI_fd(_mp_task.UI_fd)
{
	pose_list.clear();
	pose_list_iterator = pose_list.end();
}

// -------------------------------------------------------
// destruktor
mp_teach_in_generator::~mp_teach_in_generator (void) { flush_pose_list(); }




// --------------------------------------------------------------------------
// Zapis trajektorii do pliku
void mp_teach_in_generator::save_file (POSE_SPECIFICATION ps) {
	ECP_message ecp_to_ui_msg; // Przesylka z ECP do UI
	UI_reply ui_to_ecp_rep;    // Odpowiedz UI do ECP
	mp_taught_in_pose tip;        // Zapisywana pozycja
	char *cwd;                 // Wsk. na nazwe biezacego katalogu
	char coordinate_type[80];  // Opis wspolrzednych: "MOTOR", "JOINT", ...
	uint64_t e;       // Kod bledu systemowego
	uint64_t number_of_poses; // Liczba pozycji do zapamietania
	uint64_t i, j;    // Liczniki petli

	ecp_to_ui_msg.hdr.type = 0;

	ecp_to_ui_msg.ecp_message = SAVE_FILE;   // Polecenie wprowadzenia nazwy pliku
	strcpy(ecp_to_ui_msg.string, "*.trj");   // Wzorzec nazwy pliku
// if ( Send (UI_pid, &ecp_to_ui_msg, &ui_to_ecp_rep, sizeof(ECP_message), sizeof(UI_reply)) == -1) {
	if (MsgSend(UI_fd, &ecp_to_ui_msg,  sizeof(ECP_message),  &ui_to_ecp_rep, sizeof(UI_reply)) < 0) {// by Y&W
		e = errno;
		perror("ECP: Send() to UI failed\n");
		sr_ecp_msg.message (SYSTEM_ERROR, e, "ECP: Send() to UI failed");
		throw mp_generator::MP_error(SYSTEM_ERROR, (uint64_t) 0);
	}
	if ( ui_to_ecp_rep.reply == QUIT) // Nie wybrano nazwy pliku lub zrezygnowano z zapisu
		return;
	switch (ps) {
		case MOTOR:
			strcpy (coordinate_type, "MOTOR");
			break;
		case JOINT:
			strcpy (coordinate_type, "JOINT");
			break;
		case XYZ_ANGLE_AXIS:
			strcpy (coordinate_type, "XYZ_ANGLE_AXIS");
			break;
		case XYZ_EULER_ZYZ:
			strcpy (coordinate_type, "XYZ_EULER_ZYZ");
			break;
		case PF_VELOCITY:
			strcpy (coordinate_type, "PF_VELOCITY");
			break;

		default:
			strcpy (coordinate_type, "MOTOR");
	}
	cwd = getcwd (NULL, 0);
	if ( chdir(ui_to_ecp_rep.path) != 0 ) {
		perror(ui_to_ecp_rep.path);
		throw mp_generator::MP_error(NON_FATAL_ERROR, NON_EXISTENT_DIRECTORY);
	}
	ofstream to_file(ui_to_ecp_rep.filename); // otworz plik do zapisu
	e = errno;
	if (!to_file) {
		perror(ui_to_ecp_rep.filename);
		throw mp_generator::MP_error(NON_FATAL_ERROR, NON_EXISTENT_FILE);
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
			if (ps == PF_VELOCITY) { // by Y
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
bool mp_teach_in_generator::load_file_with_path (const char* file_name, short robot_number) {
// Funkcja zwraca true jesli wczytanie trajektorii powiodlo sie,
// false w przeciwnym razie
// mp_taught_in_pose tip;        // Wczytana pozycja
// char *cwd;                 // Wsk. na nazwe biezacego katalogu
	char coordinate_type[80];  // Opis wspolrzednych: "MOTOR", "JOINT", ...
	POSE_SPECIFICATION ps;     // Rodzaj wspolrzednych
	// uint64_t e;       // Kod bledu systemowego
	uint64_t number_of_poses; // Liczba zapamietanych pozycji
	uint64_t i, j;    // Liczniki petli
	bool first_time = true; // Znacznik
	double irp6ot_coordinates[MAX_SERVOS_NR];     // Wczytane wspolrzedne
	double irp6p_coordinates[MAX_SERVOS_NR];     // Wczytane wspolrzedne
	double motion_time;        // Czas dojscia do wspolrzednych


	ifstream from_file(file_name); // otworz plik do odczytu
	if (!from_file) {
		perror(file_name);
		throw mp_generator::MP_error(NON_FATAL_ERROR, NON_EXISTENT_FILE);
	}

	if ( !(from_file >> coordinate_type) ) {
		from_file.close();
		throw mp_generator::MP_error (NON_FATAL_ERROR, READ_FILE_ERROR);
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
		ps = MOTOR;
	} else if ( !strcmp(coordinate_type, "JOINT") )
		ps = JOINT;
	else if ( !strcmp(coordinate_type, "XYZ_ANGLE_AXIS") )
		ps = XYZ_ANGLE_AXIS;
	else if ( !strcmp(coordinate_type, "XYZ_EULER_ZYZ") )
		ps = XYZ_EULER_ZYZ;
	else if ( !strcmp(coordinate_type, "PF_VELOCITY") )
		ps = PF_VELOCITY;
	else {
		from_file.close();
		throw mp_generator::MP_error(NON_FATAL_ERROR, NON_TRAJECTORY_FILE);
	}

	if ( !(from_file >> number_of_poses) ) {
		from_file.close();
		throw mp_generator::MP_error (NON_FATAL_ERROR, READ_FILE_ERROR);
	}

	flush_pose_list(); // Usuniecie listy pozycji, o ile istnieje
	for ( i = 0; i < number_of_poses; i++) {

		if (!(from_file >> motion_time)) {
			from_file.close();
			throw mp_generator::MP_error (NON_FATAL_ERROR, READ_FILE_ERROR);
		}
		for ( j = 0; j < MAX_SERVOS_NR; j++) {
			if ( !(from_file >> irp6ot_coordinates[j]) ) { // Zabezpieczenie przed danymi nienumerycznymi
				from_file.close();
				throw mp_generator::MP_error (NON_FATAL_ERROR, READ_FILE_ERROR);
			}
		}
		if (robot_number > 1) {
			for ( j = 0; j < MAX_SERVOS_NR; j++) {
				if ( !(from_file >> irp6p_coordinates[j]) ) { // Zabezpieczenie przed danymi nienumerycznymi
					from_file.close();
					throw mp_generator::MP_error (NON_FATAL_ERROR, READ_FILE_ERROR);
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
	from_file.close();
	return true;
} // end: load_file()
// --------------------------------------------------------------------------




// --------------------------------------------------------------------------
// Wczytanie trajektorii z pliku
bool mp_teach_in_generator::load_file () {
// Funkcja zwraca true jesli wczytanie trajektorii powiodlo sie,
// false w przeciwnym razie
	ECP_message ecp_to_ui_msg; // Przesylka z ECP do UI
	UI_reply ui_to_ecp_rep;    // Odpowiedz UI do ECP
// mp_taught_in_pose tip;        // Wczytana pozycja
// char *cwd;                 // Wsk. na nazwe biezacego katalogu
	char coordinate_type[80];  // Opis wspolrzednych: "MOTOR", "JOINT", ...
	POSE_SPECIFICATION ps;     // Rodzaj wspolrzednych
	uint64_t e;       // Kod bledu systemowego
	uint64_t number_of_poses; // Liczba zapamietanych pozycji
	uint64_t i, j;    // Liczniki petli
	bool first_time = true; // Znacznik
	double coordinates[MAX_SERVOS_NR];     // Wczytane wspolrzedne
	int extra_info;				// by Y - dodatkowe info do dowolnego wykorzystania
	double motion_time;        // Czas dojscia do wspolrzednych

	ecp_to_ui_msg.hdr.type = 0;

	ecp_to_ui_msg.ecp_message = LOAD_FILE;   // Polecenie wprowadzenia nazwy odczytywanego pliku

// if ( Send (UI_pid, &ecp_to_ui_msg, &ui_to_ecp_rep, sizeof(ECP_message), sizeof(UI_reply)) == -1) {
	if (MsgSend(UI_fd, &ecp_to_ui_msg,  sizeof(ECP_message),  &ui_to_ecp_rep, sizeof(UI_reply)) < 0) {// by Y&W
		e = errno;
		perror("ECP: Send() to UI failed\n");
		sr_ecp_msg.message (SYSTEM_ERROR, e, "ECP: Send() to UI failed");
		throw mp_generator::MP_error(SYSTEM_ERROR, (uint64_t) 0);
	}
	if ( ui_to_ecp_rep.reply == QUIT) // Nie wybrano nazwy pliku lub zrezygnowano z zapisu
		return false;
	if ( chdir(ui_to_ecp_rep.path) != 0 ) {
		perror(ui_to_ecp_rep.path);
		throw mp_generator::MP_error(NON_FATAL_ERROR, NON_EXISTENT_DIRECTORY);
	}

	ifstream from_file(ui_to_ecp_rep.filename); // otworz plik do odczytu
	if (!from_file) {
		perror(ui_to_ecp_rep.filename);
		throw mp_generator::MP_error(NON_FATAL_ERROR, NON_EXISTENT_FILE);
	}
	if ( !(from_file >> coordinate_type) ) {
		from_file.close();
		throw mp_generator::MP_error (NON_FATAL_ERROR, READ_FILE_ERROR);
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
		ps = MOTOR;
	} else if ( !strcmp(coordinate_type, "JOINT") )
		ps = JOINT;
	else if ( !strcmp(coordinate_type, "XYZ_ANGLE_AXIS") )
		ps = XYZ_ANGLE_AXIS;
	else if ( !strcmp(coordinate_type, "XYZ_EULER_ZYZ") )
		ps = XYZ_EULER_ZYZ;
	else if ( !strcmp(coordinate_type, "PF_VELOCITY") )
		ps = PF_VELOCITY;
	else {
		from_file.close();
		throw mp_generator::MP_error(NON_FATAL_ERROR, NON_TRAJECTORY_FILE);
	}
	if ( !(from_file >> number_of_poses) ) {
		from_file.close();
		throw mp_generator::MP_error (NON_FATAL_ERROR, READ_FILE_ERROR);
	}
	flush_pose_list(); // Usuniecie listy pozycji, o ile istnieje
	for ( i = 0; i < number_of_poses; i++) {
		if (!(from_file >> motion_time)) {
			from_file.close();
			throw mp_generator::MP_error (NON_FATAL_ERROR, READ_FILE_ERROR);
		}
		for ( j = 0; j < MAX_SERVOS_NR; j++) {
			if ( !(from_file >> coordinates[j]) ) { // Zabezpieczenie przed danymi nienumerycznymi
				from_file.close();
				throw mp_generator::MP_error (NON_FATAL_ERROR, READ_FILE_ERROR);
			}
		}

		if (ps == PF_VELOCITY) { // by Y
			if ( !(from_file >> extra_info) ) { // Zabezpieczenie przed danymi nienumerycznymi
				from_file.close();
				throw mp_generator::MP_error (NON_FATAL_ERROR, READ_FILE_ERROR);
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
	from_file.close();
	return true;
} // end: load_file()
// --------------------------------------------------------------------------



// -------------------------------------------------------
void mp_teach_in_generator::flush_pose_list ( void ) {
	pose_list.clear();
}
; // end: flush_pose_list
// -------------------------------------------------------
void mp_teach_in_generator::initiate_pose_list(void) { pose_list_iterator = pose_list.begin();};
// -------------------------------------------------------
void mp_teach_in_generator::next_pose_list_ptr (void) {
	if (pose_list_iterator != pose_list.end())
		pose_list_iterator++;
}
// -------------------------------------------------------
void mp_teach_in_generator::get_pose (mp_taught_in_pose& tip) { // by Y
	tip.arm_type = pose_list_iterator->arm_type;
	tip.motion_time = pose_list_iterator->motion_time;
	tip.extra_info = pose_list_iterator->extra_info;
	memcpy(tip.coordinates, pose_list_iterator->coordinates, MAX_SERVOS_NR*sizeof(double));
	memcpy(tip.irp6p_coordinates, pose_list_iterator->irp6p_coordinates, MAX_SERVOS_NR*sizeof(double));
}
// -------------------------------------------------------
// Pobierz nastepna pozycje z listy
void mp_teach_in_generator::get_next_pose (double next_pose[MAX_SERVOS_NR])  {
	memcpy(next_pose, pose_list_iterator->coordinates, MAX_SERVOS_NR*sizeof(double));
};

void mp_teach_in_generator::get_next_pose (double next_pose[MAX_SERVOS_NR], double irp6_next_pose[MAX_SERVOS_NR])  {
	memcpy(next_pose, pose_list_iterator->coordinates, MAX_SERVOS_NR*sizeof(double));
	memcpy(irp6_next_pose, pose_list_iterator->irp6p_coordinates, MAX_SERVOS_NR*sizeof(double));
};


// -------------------------------------------------------
void mp_teach_in_generator::set_pose (POSE_SPECIFICATION ps, double motion_time, double coordinates[MAX_SERVOS_NR]) {
	pose_list_iterator->arm_type = ps;
	pose_list_iterator->motion_time = motion_time;
	memcpy(pose_list_iterator->coordinates, coordinates, MAX_SERVOS_NR*sizeof(double));
}

void mp_teach_in_generator::set_pose (POSE_SPECIFICATION ps, double motion_time, double coordinates[MAX_SERVOS_NR],
                                      double irp6p_coordinates[MAX_SERVOS_NR]) {
	pose_list_iterator->arm_type = ps;
	pose_list_iterator->motion_time = motion_time;
	memcpy(pose_list_iterator->coordinates, coordinates, MAX_SERVOS_NR*sizeof(double));
	memcpy(pose_list_iterator->irp6p_coordinates, irp6p_coordinates, MAX_SERVOS_NR*sizeof(double));
}

// -------------------------------------------------------
bool mp_teach_in_generator::is_pose_list_element ( void ) {
	// sprawdza czy aktualnie wskazywany jest element listy, czy lista sie skonczyla
	if ( pose_list_iterator != pose_list.end())
		return true;
	else
		return false;
}
// -------------------------------------------------------
bool mp_teach_in_generator::is_last_list_element ( void ) {
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
};
// -------------------------------------------------------

void mp_teach_in_generator::create_pose_list_head (POSE_SPECIFICATION ps, double motion_time, double coordinates[MAX_SERVOS_NR]) {
	pose_list.push_back(mp_taught_in_pose(ps, motion_time, coordinates));
	pose_list_iterator = pose_list.begin();
}

// by Y

void mp_teach_in_generator::create_pose_list_head (POSE_SPECIFICATION ps, double motion_time, double coordinates[MAX_SERVOS_NR],
        double irp6p_coordinates[MAX_SERVOS_NR]) {
	pose_list.push_back(mp_taught_in_pose(ps, motion_time, coordinates, irp6p_coordinates));
	pose_list_iterator = pose_list.begin();
}


void mp_teach_in_generator::create_pose_list_head (POSE_SPECIFICATION ps, double motion_time, int extra_info, double coordinates[MAX_SERVOS_NR]) {
	pose_list.push_back(mp_taught_in_pose(ps, motion_time, extra_info, coordinates));
	pose_list_iterator = pose_list.begin();
}

void mp_teach_in_generator::insert_pose_list_element (POSE_SPECIFICATION ps, double motion_time, double coordinates[MAX_SERVOS_NR]) {
	pose_list.push_back(mp_taught_in_pose(ps, motion_time, coordinates));
	pose_list_iterator++;
}

// by Y

void mp_teach_in_generator::insert_pose_list_element (POSE_SPECIFICATION ps, double motion_time, double coordinates[MAX_SERVOS_NR],
        double irp6p_coordinates[MAX_SERVOS_NR]) {
	pose_list.push_back(mp_taught_in_pose(ps, motion_time, coordinates, irp6p_coordinates));
	pose_list_iterator++;
}


void mp_teach_in_generator::insert_pose_list_element (POSE_SPECIFICATION ps, double motion_time, int extra_info, double coordinates[MAX_SERVOS_NR]) {
	pose_list.push_back(mp_taught_in_pose(ps, motion_time, extra_info, coordinates));
	pose_list_iterator++;
}

// -------------------------------------------------------
int mp_teach_in_generator::pose_list_length(void) { return pose_list.size(); };
// -------------------------------------------------------



// ----------------------------------------------------------------------------------------------
// ----------------------  metoda	first_step -------------------------------------------------
// ----------------------------------------------------------------------------------------------

bool mp_teach_in_generator::first_step () {
//  printf("w mp_teach_in_generator::first_step\n");
	idle_step_counter = 1;

//   printf("w mp_teach_in_generator::first_step 2\n");
	initiate_pose_list();

	for (map <ROBOT_ENUM, mp_robot*>::iterator robot_m_iterator = robot_m.begin();
	        robot_m_iterator != robot_m.end(); robot_m_iterator++) {
		robot_m_iterator->second->ecp_td.mp_command = NEXT_POSE;
		robot_m_iterator->second->ecp_td.instruction_type = GET;
		robot_m_iterator->second->ecp_td.get_type = ARM_DV;
		robot_m_iterator->second->ecp_td.get_arm_type = MOTOR;
		robot_m_iterator->second->communicate = true;
	}


//  printf("w mp_teach_in_generator::first_step za initiate_pose_list\n");
	return next_step ();
}
; // end: bool mp_teach_in_generator::first_step ( )

// ----------------------------------------------------------------------------------------------
// ----------------------  metoda	next_step --------------------------------------------------
// ----------------------------------------------------------------------------------------------

bool mp_teach_in_generator::next_step () {
	mp_taught_in_pose tip;      // Nauczona pozycja

// printf("W mp_teach_in_generator::next_step\n");
	if ( idle_step_counter ) { // Oczekiwanie na odczyt aktualnego polozenia koncowki
		idle_step_counter--;
		return true;
	}

// printf("W mp_teach_in_generator::next_step\n");
	/*
	  if (is_pose_list_element ())
	    the_robot.set_ecp_reply (ECP_ACKNOWLEDGE);
	  else
	    the_robot.set_ecp_reply (TASK_TERMINATED);
	*/
// printf("W mp_teach_in_generator::next_step przed mp_buffer_receive_and_send\n");
// the_robot.mp_buffer_receive_and_send ();
// printf("W mp_teach_in_generator::next_step za mp_buffer_receive_and_send\n");

	if (!is_pose_list_element ())
		return false; // Jezeli lista jest pusta to konczymy generacje trajektorii

// printf("W mp_teach_in_generator::next_step\n");

	get_pose (tip);

	map <ROBOT_ENUM, mp_robot*>::iterator robot_m_iterator = robot_m.begin();

	// Przepisanie pozycji z listy
	switch ( tip.arm_type ) {
		case C_MOTOR:
			robot_m_iterator->second->ecp_td.instruction_type = SET;
			robot_m_iterator->second->ecp_td.set_type = ARM_DV; // ARM
			robot_m_iterator->second->ecp_td.set_arm_type = MOTOR;
			robot_m_iterator->second->ecp_td.motion_type = ABSOLUTE;
			robot_m_iterator->second->ecp_td.next_interpolation_type = MIM;
			robot_m_iterator->second->ecp_td.motion_steps = (WORD) ceil(tip.motion_time / STEP);
			robot_m_iterator->second->ecp_td.value_in_step_no = robot_m_iterator->second->ecp_td.motion_steps;
			memcpy (robot_m_iterator->second->ecp_td.next_motor_arm_coordinates, tip.coordinates, MAX_SERVOS_NR*sizeof (double));
			break;
		case C_JOINT:
			robot_m_iterator->second->ecp_td.instruction_type = SET;
			robot_m_iterator->second->ecp_td.set_type = ARM_DV; // ARM
			robot_m_iterator->second->ecp_td.set_arm_type = JOINT;
			robot_m_iterator->second->ecp_td.motion_type = ABSOLUTE;
			robot_m_iterator->second->ecp_td.next_interpolation_type = MIM;
			robot_m_iterator->second->ecp_td.motion_steps = (WORD) ceil(tip.motion_time / STEP);
			robot_m_iterator->second->ecp_td.value_in_step_no = robot_m_iterator->second->ecp_td.motion_steps;
			memcpy (robot_m_iterator->second->ecp_td.next_joint_arm_coordinates, tip.coordinates, MAX_SERVOS_NR*sizeof (double));
			break;
		case C_XYZ_EULER_ZYZ:
			robot_m_iterator->second->ecp_td.instruction_type = SET;
			robot_m_iterator->second->ecp_td.set_type = ARM_DV; // ARM
			robot_m_iterator->second->ecp_td.set_arm_type = XYZ_EULER_ZYZ;
			robot_m_iterator->second->ecp_td.motion_type = ABSOLUTE;
			robot_m_iterator->second->ecp_td.next_interpolation_type = MIM;
			robot_m_iterator->second->ecp_td.motion_steps = (WORD) ceil(tip.motion_time / STEP);
			robot_m_iterator->second->ecp_td.value_in_step_no = robot_m_iterator->second->ecp_td.motion_steps;
			memcpy (robot_m_iterator->second->ecp_td.next_XYZ_ZYZ_arm_coordinates, tip.coordinates, 6*sizeof (double));
			robot_m_iterator->second->ecp_td.next_gripper_coordinate = tip.coordinates[6];
			break;
		case C_XYZ_ANGLE_AXIS:
			robot_m_iterator->second->ecp_td.instruction_type = SET;
			robot_m_iterator->second->ecp_td.set_type = ARM_DV; // ARM
			robot_m_iterator->second->ecp_td.set_arm_type = XYZ_ANGLE_AXIS;
			robot_m_iterator->second->ecp_td.motion_type = ABSOLUTE;
			robot_m_iterator->second->ecp_td.next_interpolation_type = MIM;
			robot_m_iterator->second->ecp_td.motion_steps = (WORD) ceil(tip.motion_time / STEP);
			robot_m_iterator->second->ecp_td.value_in_step_no = robot_m_iterator->second->ecp_td.motion_steps;
			memcpy (robot_m_iterator->second->ecp_td.next_XYZ_AA_arm_coordinates, tip.coordinates, 6*sizeof (double));
			robot_m_iterator->second->ecp_td.next_gripper_coordinate = tip.coordinates[6];
			break;
		default:
			throw MP_error (NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
	} // end: switch
	if ((++robot_m_iterator) != robot_m.end()) {
		//       	printf("postument\n");
		switch ( tip.arm_type ) {
			case C_MOTOR:
				robot_m_iterator->second->ecp_td.instruction_type = SET;
				robot_m_iterator->second->ecp_td.set_type = ARM_DV; // ARM
				robot_m_iterator->second->ecp_td.set_arm_type = MOTOR;
				robot_m_iterator->second->ecp_td.motion_type = ABSOLUTE;
				robot_m_iterator->second->ecp_td.next_interpolation_type = MIM;
				robot_m_iterator->second->ecp_td.motion_steps = (WORD) ceil(tip.motion_time / STEP);
				robot_m_iterator->second->ecp_td.value_in_step_no = robot_m_iterator->second->ecp_td.motion_steps;
				memcpy (robot_m_iterator->second->ecp_td.next_motor_arm_coordinates, tip.irp6p_coordinates, MAX_SERVOS_NR*sizeof (double));
				break;
			case C_JOINT:
				robot_m_iterator->second->ecp_td.instruction_type = SET;
				robot_m_iterator->second->ecp_td.set_type = ARM_DV; // ARM
				robot_m_iterator->second->ecp_td.set_arm_type = JOINT;
				robot_m_iterator->second->ecp_td.motion_type = ABSOLUTE;
				robot_m_iterator->second->ecp_td.next_interpolation_type = MIM;
				robot_m_iterator->second->ecp_td.motion_steps = (WORD) ceil(tip.motion_time / STEP);
				robot_m_iterator->second->ecp_td.value_in_step_no = robot_m_iterator->second->ecp_td.motion_steps;
				memcpy (robot_m_iterator->second->ecp_td.next_joint_arm_coordinates, tip.irp6p_coordinates, MAX_SERVOS_NR*sizeof (double));
				break;
			case C_XYZ_EULER_ZYZ:
				robot_m_iterator->second->ecp_td.instruction_type = SET;
				robot_m_iterator->second->ecp_td.set_type = ARM_DV; // ARM
				robot_m_iterator->second->ecp_td.set_arm_type = XYZ_EULER_ZYZ;
				robot_m_iterator->second->ecp_td.motion_type = ABSOLUTE;
				robot_m_iterator->second->ecp_td.next_interpolation_type = MIM;
				robot_m_iterator->second->ecp_td.motion_steps = (WORD) ceil(tip.motion_time / STEP);
				robot_m_iterator->second->ecp_td.value_in_step_no = robot_m_iterator->second->ecp_td.motion_steps;
				memcpy (robot_m_iterator->second->ecp_td.next_XYZ_ZYZ_arm_coordinates, tip.irp6p_coordinates, 6*sizeof (double));
				robot_m_iterator->second->ecp_td.next_gripper_coordinate = tip.irp6p_coordinates[6];
				break;
			case C_XYZ_ANGLE_AXIS:
				robot_m_iterator->second->ecp_td.instruction_type = SET;
				robot_m_iterator->second->ecp_td.set_type = ARM_DV; // ARM
				robot_m_iterator->second->ecp_td.set_arm_type = XYZ_ANGLE_AXIS;
				robot_m_iterator->second->ecp_td.motion_type = ABSOLUTE;
				robot_m_iterator->second->ecp_td.next_interpolation_type = MIM;
				robot_m_iterator->second->ecp_td.motion_steps = (WORD) ceil(tip.motion_time / STEP);
				robot_m_iterator->second->ecp_td.value_in_step_no = robot_m_iterator->second->ecp_td.motion_steps;
				memcpy (robot_m_iterator->second->ecp_td.next_XYZ_AA_arm_coordinates, tip.irp6p_coordinates, 6*sizeof (double));
				robot_m_iterator->second->ecp_td.next_gripper_coordinate = tip.irp6p_coordinates[6];
				break;
			default:
				throw MP_error (NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
		} // end: switch
	}


	next_pose_list_ptr (); // nastepna pozycja
	return true;

}
; // end: bool mp_teach_in_generator::next_step ( )
