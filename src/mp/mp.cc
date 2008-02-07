// -------------------------------------------------------------------------
//                              mp.cc
//
// MP Master Process - methods
//
// Ostatnia modyfikacja: 2007
// -------------------------------------------------------------------------
// Funkcje do konstruowania procesow MP

#include <unistd.h>

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

mp_taught_in_pose:: mp_taught_in_pose (void) {};
mp_taught_in_pose::mp_taught_in_pose (POSE_SPECIFICATION at, double mt, double* c)
		: arm_type(at), motion_time(mt) {
	memcpy(coordinates, c, MAX_SERVOS_NR*sizeof(double));
}
; // end: mp_taught_in_pose::mp_taught_in_pose

mp_taught_in_pose::mp_taught_in_pose (POSE_SPECIFICATION at, double mt, double* c, double* irp6p_c)
		: arm_type(at), motion_time(mt) {
	memcpy(coordinates, c, MAX_SERVOS_NR*sizeof(double));
	memcpy(irp6p_coordinates, irp6p_c, MAX_SERVOS_NR*sizeof(double));
}
; // end: mp_taught_in_pose::mp_taught_in_pose

mp_taught_in_pose::mp_taught_in_pose (POSE_SPECIFICATION at, double mt, int e_info, double* c)
		: arm_type(at), motion_time(mt) { // by Y
	memcpy(coordinates, c, MAX_SERVOS_NR*sizeof(double));
	extra_info = e_info;
}
; // end: mp_taught_in_pose::mp_taught_in_pose

robot_ECP_transmission_data::robot_ECP_transmission_data (void)
		: instruction_type(INVALID), reply_type(ACKNOWLEDGE) {}; // konstruktor



mp_robot::MP_error::MP_error (uint64_t err0, uint64_t err1)
 : error_class(err0), mp_error(err1)
{}

mp_generator::MP_error::MP_error (uint64_t err0, uint64_t err1)
 : error_class(err0), mp_error(err1)
{}


mp_generator::mp_generator(mp_task& _mp_task) : mp_t(_mp_task) {
	wait_for_ECP_pulse = false;
	sr_ecp_msg = mp_t.sr_ecp_msg;
	trigger = false;
	phase = BEFORE_FIRST_STEP;
	new_pulse_checked = true;
};


mp_generator::~mp_generator(void) { };

void mp_generator::re_run(void) // powrot do stanu wyjsciowego
{
	phase = BEFORE_FIRST_STEP;
	new_pulse_checked = true;
}

// kopiuje dane z robotow do generatora
void mp_generator::copy_data(map <ROBOT_ENUM, mp_robot*>& _robot_m) {
	for (map <ROBOT_ENUM, mp_robot*>::iterator robot_m_iterator = _robot_m.begin();
	        robot_m_iterator != _robot_m.end(); robot_m_iterator++) {
		robot_m_iterator->second->get_reply(); // odpowiedz ECP
	}
};

// kopiuje polecenie stworzone w generatorze do robotow
void mp_generator::copy_generator_command (map <ROBOT_ENUM, mp_robot*>& _robot_m) {
	for (map <ROBOT_ENUM, mp_robot*>::iterator robot_m_iterator = _robot_m.begin();
	        robot_m_iterator != _robot_m.end(); robot_m_iterator++) {
		robot_m_iterator->second->create_command(); // odpowiedz ECP
	}
};




// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// ##############################################################
// ##############################################################
//                              CIALA METOD dla generatorow
// ##############################################################
// ##############################################################
// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


// condition to wait for desired time in ms

mp_delay_ms_condition::mp_delay_ms_condition(mp_task& _mp_task, int _ms_delay): mp_generator (_mp_task) {
	local_timer = new mp_timer();
	configure(_ms_delay);
};

mp_delay_ms_condition::~mp_delay_ms_condition() {
	delete local_timer;
};


void mp_delay_ms_condition::configure (int _ms_delay) {
	ms_delay = _ms_delay;
};

// ----------------------------------------------------------------------------------------------
// ---------------------------------    metoda	first_step -------------------------------------
// ----------------------------------------------------------------------------------------------

bool mp_delay_ms_condition::first_step () {
	local_timer->timer_start(NULL);
	return true;
}
; // end: mp_set_next_ecps_state_generator::first_step()

// ----------------------------------------------------------------------------------------------
// -----------------------------------  metoda	next_step -----------------------------------
// ----------------------------------------------------------------------------------------------

bool mp_delay_ms_condition::next_step () {
	local_timer->timer_stop(NULL);
	local_timer->get_time(&sec);
	if (1000*sec > (float) ms_delay) return false;
	delay (20);
	local_timer->timer_stop(NULL);
	local_timer->get_time(&sec);
	if (1000*sec > (float) ms_delay) return false;
	return true;
}
; // end: bool mp_set_next_ecps_state_generator::next_step ()


// generator for setting the next ecps state


mp_set_next_ecps_state_generator::mp_set_next_ecps_state_generator(mp_task& _mp_task):
	mp_generator (_mp_task) {};


void mp_set_next_ecps_state_generator::configure (int l_mp_2_ecp_next_state, int l_mp_2_ecp_next_state_variant,
        char* l_mp_2_ecp_next_state_string) {
	mp_2_ecp_next_state = l_mp_2_ecp_next_state;
	mp_2_ecp_next_state_variant = l_mp_2_ecp_next_state_variant;
	strcpy (mp_2_ecp_next_state_string, l_mp_2_ecp_next_state_string);
};


// ----------------------------------------------------------------------------------------------
// ---------------------------------    metoda	first_step -------------------------------------
// ----------------------------------------------------------------------------------------------

bool mp_set_next_ecps_state_generator::first_step () {
	for (map <ROBOT_ENUM, mp_robot*>::iterator robot_m_iterator = robot_m.begin();
	        robot_m_iterator != robot_m.end(); robot_m_iterator++) {
		robot_m_iterator->second->ecp_td.mp_command = NEXT_STATE;
		robot_m_iterator->second->ecp_td.mp_2_ecp_next_state = mp_2_ecp_next_state;
		robot_m_iterator->second->ecp_td.mp_2_ecp_next_state_variant = mp_2_ecp_next_state_variant;
		strcpy (robot_m_iterator->second->ecp_td.mp_2_ecp_next_state_string, mp_2_ecp_next_state_string);
		robot_m_iterator->second->communicate = true;
	}

	copy_generator_command( robot_m );
	return true;
}
; // end: mp_set_next_ecps_state_generator::first_step()

// ----------------------------------------------------------------------------------------------
// -----------------------------------  metoda	next_step -----------------------------------
// ----------------------------------------------------------------------------------------------

bool mp_set_next_ecps_state_generator::next_step () {
	copy_data( robot_m ); // Kopiowanie danych z bufora przyslanego z ECP do
	return false;
}
; // end: bool mp_set_next_ecps_state_generator::next_step ()

mp_send_end_motion_to_ecps_generator::mp_send_end_motion_to_ecps_generator(mp_task& _mp_task): mp_generator (_mp_task) {};


// ----------------------------------------------------------------------------------------------
// ---------------------------------    metoda	first_step -------------------------------------
// ----------------------------------------------------------------------------------------------

bool mp_send_end_motion_to_ecps_generator::first_step () {
	for (map <ROBOT_ENUM, mp_robot*>::iterator robot_m_iterator = robot_m.begin();
	        robot_m_iterator != robot_m.end(); robot_m_iterator++) {
		robot_m_iterator->second->ecp_td.mp_command = END_MOTION;
		robot_m_iterator->second->communicate = true;
	}

	copy_generator_command( robot_m );
	return true;
}
; // end: mp_send_end_motion_to_ecps_state_generator::first_step()

// ----------------------------------------------------------------------------------------------
// -----------------------------------  metoda	next_step -----------------------------------
// ----------------------------------------------------------------------------------------------

bool mp_send_end_motion_to_ecps_generator::next_step () {
	copy_data( robot_m ); // Kopiowanie danych z bufora przyslanego z ECP do
	return false;
}
; // end: bool mp_send_end_motion_to_ecps_state_generator::next_step ()





// ###############################################################
// Rozszerzony generator pusty. Faktyczna generacja trajektorii odbywa sie w ECP
// ###############################################################


mp_extended_empty_generator::mp_extended_empty_generator(mp_task& _mp_task):
		mp_generator (_mp_task) { activate_trigger = true; };


void mp_extended_empty_generator::configure (bool l_activate_trigger) {
	activate_trigger = l_activate_trigger;
}

// ----------------------------------------------------------------------------------------------
// ---------------------------------    metoda	first_step -------------------------------------
// ----------------------------------------------------------------------------------------------

bool mp_extended_empty_generator::first_step () {


// Funkcja zwraca false gdy koniec generacji trajektorii
// Funkcja zwraca true gdy generacja trajektorii bedzie kontynuowana
// Inicjacja generatora trajektorii
// printf("mp first step\n");
//	if (debug_tmp) printf(" mp_extended_empty_generator first_step\n");
	wait_for_ECP_pulse = true;
	for (map <ROBOT_ENUM, mp_robot*>::iterator robot_m_iterator = robot_m.begin();
	        robot_m_iterator != robot_m.end(); robot_m_iterator++) {
		robot_m_iterator->second->ecp_td.mp_command = NEXT_POSE;
		robot_m_iterator->second->ecp_td.instruction_type = QUERY;
		robot_m_iterator->second->communicate = false;

	}
//	if (debug_tmp) printf(" mp_extended_empty_generator first_step 4\n");
	copy_generator_command( robot_m );
// 	if (debug_tmp) printf(" mp_extended_empty_generator first_step 5\n");
	return true;
}
; // end: mp_extended_empty_generator::first_step()

// ----------------------------------------------------------------------------------------------
// -----------------------------------  metoda	next_step --------------------------------------
// ----------------------------------------------------------------------------------------------

bool mp_extended_empty_generator::next_step () {
// Funkcja zwraca false gdy koniec generacji trajektorii
// Funkcja zwraca true gdy generacja trajektorii bedzie kontynuowana
// Na podstawie ecp_reply dla poszczegolnych robotow nalezy okreslic czy
// skonczono zadanie uzytkownika

	copy_data( robot_m ); // Kopiowanie danych z bufora przyslanego z ECP do
	// obrazu danych wykorzystywanych przez generator

//	if (debug_tmp) printf(" mp_extended_empty_generator next_step\n");

// 	if (trigger) printf("Yh\n"); else printf("N\n");
// printf("mp next step\n");
// UWAGA: dzialamy na jednoelementowej liscie robotow

	if (trigger && activate_trigger) {
		return false;
	}

	for (map <ROBOT_ENUM, mp_robot*>::iterator robot_m_iterator = robot_m.begin();
	        robot_m_iterator != robot_m.end(); robot_m_iterator++) {
		if (robot_m_iterator->second->new_pulse) {
			// printf("mpextempty_gen r: %d, pc: %d\n", robot_m_iterator->first, robot_m_iterator->second->pulse_code);
			robot_m_iterator->second->communicate = true;
		} else {
			robot_m_iterator->second->communicate = false;
		}
	}


	for (map <ROBOT_ENUM, mp_robot*>::iterator robot_m_iterator = robot_m.begin();
	        robot_m_iterator != robot_m.end(); robot_m_iterator++) {
		if ( robot_m_iterator->second->ecp_td.ecp_reply == TASK_TERMINATED ) {
			sr_ecp_msg->message("w mp task terminated");
			return false;
		}

	}

	return true;

}
; // end:  mp_extended_empty_generator::next_step()







// ###############################################################
// Generator pusty. Faktyczna generacja trajektorii odbywa sie w ECP
// ###############################################################


mp_empty_generator::mp_empty_generator(mp_task& _mp_task): mp_generator (_mp_task) {};


// ----------------------------------------------------------------------------------------------
// ---------------------------------    metoda	first_step -------------------------------------
// ----------------------------------------------------------------------------------------------

bool mp_empty_generator::first_step () {


// Funkcja zwraca false gdy koniec generacji trajektorii
// Funkcja zwraca true gdy generacja trajektorii bedzie kontynuowana
// Inicjacja generatora trajektorii
// printf("mp first step\n");
// wait_for_ECP_pulse = true;
	for (map <ROBOT_ENUM, mp_robot*>::iterator robot_m_iterator = robot_m.begin();
	        robot_m_iterator != robot_m.end(); robot_m_iterator++) {
		robot_m_iterator->second->ecp_td.mp_command = NEXT_POSE;
		robot_m_iterator->second->ecp_td.instruction_type = QUERY;
		robot_m_iterator->second->communicate = true;
	}

	copy_generator_command( robot_m );

	return true;
}
; // end: mp_empty_generator::first_step()

// ----------------------------------------------------------------------------------------------
// -----------------------------------  metoda	next_step --------------------------------------
// ----------------------------------------------------------------------------------------------

bool mp_empty_generator::next_step () {
// Funkcja zwraca false gdy koniec generacji trajektorii
// Funkcja zwraca true gdy generacja trajektorii bedzie kontynuowana
// Na podstawie ecp_reply dla poszczegolnych robotow nalezy okreslic czy
// skonczono zadanie uzytkownika

	copy_data( robot_m ); // Kopiowanie danych z bufora przyslanego z ECP do
	// obrazu danych wykorzystywanych przez generator

// 	if (trigger) printf("Yh\n"); else printf("N\n");
// printf("mp next step\n");
// UWAGA: dzialamy na jednoelementowej liscie robotow

	for (map <ROBOT_ENUM, mp_robot*>::iterator robot_m_iterator = robot_m.begin();
	        robot_m_iterator != robot_m.end(); robot_m_iterator++) {
		if ( robot_m_iterator->second->ecp_td.ecp_reply == TASK_TERMINATED ) {
			sr_ecp_msg->message("w mp task terminated");
			return false;
		}

	}

	return true;

}
; // end:  mp_empty_generator::next_step()



mp_delta_generator::mp_delta_generator(mp_task& _mp_task): mp_generator (_mp_task) {};


// ####################################################################################################
// Generator prostoliniowy o zadany przyrost polozenia/orientacji
// ####################################################################################################


mp_tight_coop_generator::mp_tight_coop_generator(mp_task& _mp_task, trajectory_description irp6ot_tr_des,
        trajectory_description irp6p_tr_des): mp_delta_generator (_mp_task) {    irp6ot_td = irp6ot_tr_des; irp6p_td = irp6p_tr_des;	  };
// destruktor
mp_tight_coop_generator::~mp_tight_coop_generator() { };



// ----------------------------------------------------------------------------------------------
// ---------------------------------    metoda	first_step -------------------------------------
// ----------------------------------------------------------------------------------------------

bool mp_tight_coop_generator::first_step () {
	// Generacja trajektorii prostoliniowej o zadany przyrost polozenia i orientacji
	// Funkcja zwraca false gdy koniec generacji trajektorii
	// Funkcja zwraca true gdy generacja trajektorii bedzie kontynuowana
	node_counter = 0;
	idle_step_counter = 2;


	for (map <ROBOT_ENUM, mp_robot*>::iterator robot_m_iterator = robot_m.begin();
	        robot_m_iterator != robot_m.end(); robot_m_iterator++) {
		robot_m_iterator->second->ecp_td.mp_command = NEXT_POSE;
		robot_m_iterator->second->ecp_td.instruction_type = GET;
		robot_m_iterator->second->ecp_td.get_type = ARM_DV;
		robot_m_iterator->second->ecp_td.set_type = ARM_DV;
		robot_m_iterator->second->ecp_td.set_arm_type = XYZ_EULER_ZYZ;
		robot_m_iterator->second->ecp_td.get_arm_type = XYZ_EULER_ZYZ;
		robot_m_iterator->second->ecp_td.motion_type = ABSOLUTE;
		robot_m_iterator->second->ecp_td.motion_steps = irp6ot_td.internode_step_no;
		robot_m_iterator->second->ecp_td.value_in_step_no = irp6ot_td.value_in_step_no;
		robot_m_iterator->second->communicate = true;
	}

	copy_generator_command( robot_m );
	return true;
}
; // end: tight_coop_generator::first_step()


// ----------------------------------------------------------------------------------------------
// -----------------------------------  metoda	next_step --------------------------------------
// ----------------------------------------------------------------------------------------------

bool mp_tight_coop_generator::next_step () {
// Generacja trajektorii prostoliniowej o zadany przyrost polozenia i orientacji
// Funkcja zwraca false gdy koniec generacji trajektorii
// Funkcja zwraca true gdy generacja trajektorii bedzie kontynuowana
// UWAGA: dzialamy na jednoelementowej liscie robotow
	int i; // licznik kolejnych wspolrzednych wektora [0..6]

	if ( idle_step_counter ) { // Oczekiwanie na odczyt aktualnego polozenia koncowki
		idle_step_counter--;
		return true;
	}

	if (node_counter == irp6ot_td.interpolation_node_no)
		return false;

	// Kopiowanie danych z bufora przyslanego z ECP do
	// obrazu danych wykorzystywanych przez generator
	copy_data( robot_m );

	map <ROBOT_ENUM, mp_robot*>::iterator robot_m_iterator = robot_m.begin();

	// Przygotowanie kroku ruchu - do kolejnego wezla interpolacji
	robot_m_iterator->second->ecp_td.instruction_type = SET;
	robot_m_iterator->second->ecp_td.get_type = NOTHING_DV;
	robot_m_iterator->second->ecp_td.get_arm_type = INVALID_END_EFFECTOR;
	node_counter++;
	// Obliczenie zadanej pozycji posredniej w tym kroku ruchu
	// (okreslenie kolejnego wezla interpolacji)
	for (i = 0; i < 6; i++) // zakladamy, ze na liscie jest jeden robot
		robot_m_iterator->second->ecp_td.next_XYZ_ZYZ_arm_coordinates[i] =
		    robot_m_iterator->second->ecp_td.current_XYZ_ZYZ_arm_coordinates[i]
		    + node_counter * irp6ot_td.coordinate_delta[i] / irp6ot_td.interpolation_node_no;
	// printf("X_d= %lf  X_a= %lf\n",robot_list->E_ptr->ecp_td.next_XYZ_ZYZ_arm_coordinates[0],robot_list->E_ptr->ecp_td.current_XYZ_ZYZ_arm_coordinates[0]);
	// printf("Y_d= %lf  Y_a= %lf\n",robot_list->E_ptr->ecp_td.next_XYZ_ZYZ_arm_coordinates[1],robot_list->E_ptr->ecp_td.current_XYZ_ZYZ_arm_coordinates[1]);
	// printf("Z_d= %lf  Z_a= %lf\n",robot_list->E_ptr->ecp_td.next_XYZ_ZYZ_arm_coordinates[2],robot_list->E_ptr->ecp_td.current_XYZ_ZYZ_arm_coordinates[2]);

	robot_m_iterator->second->ecp_td.next_gripper_coordinate =
	    robot_m_iterator->second->ecp_td.current_gripper_coordinate
	    + node_counter * irp6ot_td.coordinate_delta[6] / irp6ot_td.interpolation_node_no;

	// by Y - ZAKOMENTOWANE ponizej - nie wiadomo jaka idea temu przyswiecala
	// ale dzialalo to zle z generatorami transparentnymi ECP
	/*
	  if (node_counter == td.interpolation_node_no) {
	    // Zakonczenie generacji trajektorii
	    robot_list->E_ptr->ecp_td.mp_command = END_MOTION; 
	  }
	*/

	if ((++robot_m_iterator) != robot_m.end()) {
		// Przygotowanie kroku ruchu - do kolejnego wezla interpolacji
		robot_m_iterator->second->ecp_td.instruction_type = SET;
		robot_m_iterator->second->ecp_td.get_type = NOTHING_DV;
		robot_m_iterator->second->ecp_td.get_arm_type = INVALID_END_EFFECTOR;
		// Obliczenie zadanej pozycji posredniej w tym kroku ruchu
		// (okreslenie kolejnego wezla interpolacji)
		for (i = 0; i < 6; i++) // zakladamy, ze na liscie jest jeden robot
			robot_m_iterator->second->ecp_td.next_XYZ_ZYZ_arm_coordinates[i] =
			    robot_m_iterator->second->ecp_td.current_XYZ_ZYZ_arm_coordinates[i]
			    + node_counter * irp6p_td.coordinate_delta[i] / irp6p_td.interpolation_node_no;


		robot_m_iterator->second->ecp_td.next_gripper_coordinate =
		    robot_m_iterator->second->ecp_td.current_gripper_coordinate
		    + node_counter * irp6p_td.coordinate_delta[6] / irp6p_td.interpolation_node_no;


	}



	// skopiowac przygotowany rozkaz dla ECP do bufora wysylkowego
	copy_generator_command( robot_m );
	return true;
}
; // end: bool tight_coop_generator::next_step ()
