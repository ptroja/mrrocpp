// -------------------------------------------------------------------------
// Proces: 	EFFECTOR CONTROL PROCESS (ECP)
// Plik:			ecp_rsc.cc
// System:	QNX/MRROC++  v. 6.3
// Opis:		trajectory_reproduce_generator - definicja metod klasy
// 				condition - sprawdzanie, czy robot sie zatrzymal
// Autor:		tkornuta
// Data:		28.11.2005 -
// -------------------------------------------------------------------------

/********************************* INCLUDES *********************************/
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <fstream>

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"

#include "lib/srlib.h"

#include "ecp/irp6_on_track/ecp_local.h"

#include "ecp/irp6_on_track/ecp_g_rsc.h"
#include "ecp_mp/ecp_mp_s_force.h"
#include "ecp_mp/ecp_mp_s_digital_scales.h"

/*************************** CONDITION VALUE ********************************/
bool robot_stopped_condition::first_step (){
	communicate_with_edp=false;
	return true;
	};

bool robot_stopped_condition::next_step (){
	double first_position[6], second_position[6];
	bool stoped;
	do{
		// Pobranie polozenia robota.
		get_current_position(first_position);
		// Odczekanie 10 ms.
		usleep(1000*10);
		// Pobranie polozenia robota.
		get_current_position(second_position);
		stoped = false;
		// Porownanie polozen.
		for(int i=0; i<6; i++)
			if (first_position[i] != second_position[i])
				stoped = false;
		}while(!stoped);
	// Czujnik - linialy.
	ecp_mp_digital_scales_sensor* dss = (ecp_mp_digital_scales_sensor*)(sensor_m[SENSOR_DIGITAL_SCALE_SENSOR]);
	// Odebranie odczytow ostatniego polozenia i dodanie ich do listy.
	add_rse_element(*dss);
	// Czujnik sily.
	if (sensor_m.count(SENSOR_FORCE_ON_TRACK) > 0){
		ecp_mp_force_sensor* fs = (ecp_mp_force_sensor*)(sensor_m[SENSOR_FORCE_ON_TRACK]);
		// Pobranie ostatniego odczytu z czujnika sily.
		fs->get_reading();
		// Przepisanie obecnego polozenia robota do bufora w zaleznosci od rodzaju wspolrzednych.
		memcpy(last_force_sensor_reading, fs->image.sensor_union.force.rez, 6*sizeof(double));
		};
	// Odswiezenie okna.
	refresh_window();
	return false;
	};

/********************* PREPARE CONDITION FOR MOVE **************************/
void robot_stopped_condition::prepare_condition_for_motion(void){
	// Wyczyszczenie listy RSE.
	flush_rse_list();
	};

/************************* GET CURRENT POSITION *****************************/
void robot_stopped_condition::get_current_position(double current_position[6]){
	// Odczytanie polozenia robota
	// Przygotowanie rozkazu dla EDP.
	the_robot->EDP_data.instruction_type = GET;
	the_robot->EDP_data.get_type = ARM_DV; // ARM
	// Sprawdzenie rodzaju ramienia.
	the_robot->EDP_data.get_arm_type = MOTOR;
	// Przepisanie rozkazu do bufora wysylkowego.
	the_robot->create_command();
	// Zlecenie ruchu robota.
	communicate_with_edp=true;
	the_robot->execute_motion();
	communicate_with_edp=false;
	// Odebranie danych.
	the_robot->get_reply();
	// Przepisanie obecnego polozenia robota do bufora w zaleznosci od rodzaju wspolrzednych.
	memcpy(current_position, the_robot->EDP_data.current_motor_arm_coordinates, 6*sizeof(double));
 	}; // end: get_current_position

/**************************** REFRESH WINDOW *******************************/
void robot_stopped_condition::refresh_window(void){
	// Wiadomosc wysylana do UI.
	ECP_message ecp_ui_msg;
	// Odswiezenie okna.
	ecp_ui_msg.hdr.type=0;
	// Sprawdzenie, czy element nie jest pusty -> Wykonano ruch do pozycji zerowej.
	if (is_rse_list_element()){
		// Polecenie odswiezenia okna.
		ecp_ui_msg.ecp_message = TR_REFRESH_WINDOW;
		// Odczytanie ostatniego elementu.
		get_rse_list_data(ecp_ui_msg.R2S.robot_position, ecp_ui_msg.R2S.digital_scales_sensor_reading);
		// Przepisanie ostatniego odczytu czujnika sily.
		memcpy(ecp_ui_msg.R2S.force_sensor_reading, last_force_sensor_reading, 6*sizeof(double));
		for(int j=0; j<6; j++)
			printf("%f\t",ecp_ui_msg.R2S.force_sensor_reading[j]);
		printf("\n");

		// Wyslanie polecenia do UI -> Polecenie odswiezenia okna.
#if !defined(USE_MESSIP_SRR)
		if (MsgSend(UI_fd, &ecp_ui_msg, sizeof(ECP_message), NULL, 0) < 0){
#else
		int32_t answer;
		if (messip_send(UI_fd, 0, 0, &ecp_ui_msg, sizeof(ECP_message), &answer, NULL, 0, MESSIP_NOTIMEOUT) < 0){
#endif
			 perror("ECP trajectory_reproduce_thread(): Send() to UI failed");
			 sr_ecp_msg.message (SYSTEM_ERROR, errno, "ECP: Send() to UI failed");
			};
		}; // end: is_pose_list_element
	}; // end: refresh_window


/***************************** ADD RSE ELEMENT *******************************/
void robot_stopped_condition::add_rse_element(ecp_mp_digital_scales_sensor& the_sensor){
	// double current_position[6];
	// Pobranie pozycji robota.
	// Rozkaz dla EDP.
	the_robot->EDP_data.instruction_type = GET;
	the_robot->EDP_data.get_type = ARM_DV; // ARM
	the_robot->EDP_data.get_arm_type = MOTOR;
	the_robot->EDP_data.motion_steps = 1;
	the_robot->EDP_data.value_in_step_no = the_robot->EDP_data.motion_steps;
	// Stworzenie rozkazu.
	the_robot->create_command();
	// Wyslanie rozkazu.
	communicate_with_edp=true;
	the_robot->execute_motion();
	communicate_with_edp=false;
	// Odebranie odczytow polozenia.
	the_robot->get_reply();
	// Pobranie odczytow z linialow.
	the_sensor.initiate_reading();
	the_sensor.get_reading();
	// Dodanie elementu do listy
	if (rse_list.empty()){
		// Jesli glowa pusta.
		create_rse_list_head(
			the_robot->EDP_data.current_motor_arm_coordinates,
			the_sensor.image.sensor_union.ds.readings);
	}else{
		// Jesli nastepny element.
		insert_rse_list_element(
			the_robot->EDP_data.current_motor_arm_coordinates,
			the_sensor.image.sensor_union.ds.readings);
		}; // end else
	// Wyswietlenie dodanego elementu.
/*	trajectory_reproduce_element tre;
	get_tre_list_element(tre);
	int i;
	printf("Robot :: ");
	for(i=0; i<6; i++)
		printf("%f\t", tre.robot_position[i]);
	printf("\nCzujnik :: ");
	for(i=0; i<6; i++)
		printf("%f\t", tre.sensor_reading[i]);
	printf("\n");
*/	};

/**************************** SAVE RSE ELEMENTS ******************************/
void robot_stopped_condition::save_rse_list(char* filename)
{
	int i;
	// Element listy.
	robot_position_digital_scales_reading_element rse;
	// Sprawdzenie, czy lista nie jest pusta.
	if (rse_list_length() == 0) {
		sr_ecp_msg.message("TRE list empty.");
		return;
	}
	try {
		// Otworzenie pliku.
		std::ofstream to_file(filename);
		if (!to_file)
		throw ECP_main_error(FATAL_ERROR, SAVE_FILE_ERROR);
		// Przejscie na poczatek listy.
		initiate_rse_list();
		// Zapisywanie kolejnych elementow.
		while (!is_rse_list_last_element()) {
			// Pobranie elementu.
			get_rse_list_element (rse);
			// Zapis polozenia robota.
			for(i=0; i<6; i++)
			to_file << rse.robot_position[i] << ' ';
			// Zapis odczytow z czujnika
			for(i=0; i<6; i++)
			to_file << rse.sensor_reading[i] << ' ';
			// Nastepna linia.
			to_file << '\n';
			// Nastepna pozycja.
			next_rse_list_element();
		}
		// zapisanie ostatniego elementu
		get_rse_list_element (rse);
		// Zapis polozenia robota.
		for(i=0; i<6; i++)
		to_file << rse.robot_position[i] << ' ';
		// Zapis odczytow z czujnika
		for(i=0; i<6; i++)
		to_file << rse.sensor_reading[i] << ' ';
		// Nastepna linia.
		to_file << '\n';
		// Zamkniecie pliku.
		to_file.close();
		// Komentarz - zapisanie pliku.
		sr_ecp_msg.message ("Measures saved properly to file");
	} // end: TRY
	catch(ECP_main_error e) {
		// Wylapanie i oblsuga bledow.
		sr_ecp_msg.message (e.error_class, e.error_no);
	}
}

/*****************************  KONSTRUKTOR *********************************/
robot_stopped_condition::robot_stopped_condition(ecp_task& _ecp_task) :
	ecp_generator(_ecp_task)
{
	// Ustawienie elementow list na NULL.
	rse_list.clear();
	UI_fd = _ecp_task.UI_fd;

}


/******************************  DESTRUKTOR **********************************/
robot_stopped_condition::~robot_stopped_condition(void)
{
	// Usuniecie elementow z listy TRE.
	flush_rse_list();
}

/*********************** METODY ZWIAZANE Z LISTA RSE ************************/
void robot_stopped_condition::flush_rse_list(void)
{
	// Jezeli sa jakies elementy
	rse_list.clear();
}
; // end: flush_tre_list

void robot_stopped_condition::initiate_rse_list(void)
{
	rse_list_iterator = rse_list.begin();
}
;

void robot_stopped_condition::next_rse_list_element(void)
{
	// Przejscie na nastepny element.
	rse_list_iterator++;
}
;

void robot_stopped_condition::get_rse_list_element(robot_position_digital_scales_reading_element& rse)
{
	// Przepisanie pozycji robota.
	memcpy(rse.robot_position, rse_list_iterator->robot_position, 6
			*sizeof(double));
	// Przepisanie odczytow z czujnikow.
	memcpy(rse.sensor_reading, rse_list_iterator->sensor_reading, 6
			*sizeof(double));
}

void robot_stopped_condition::get_rse_list_data(double robot_position[6], double sensor_reading[6])
{
	// Przepisanie pozycji robota.
	memcpy(robot_position, rse_list_iterator->robot_position, 6*sizeof(double));
	// Przepisanie odczytow z czujnikow.
	memcpy(sensor_reading, rse_list_iterator->sensor_reading, 6*sizeof(double));
}

bool robot_stopped_condition::is_rse_list_element(void)
{
	// sprawdza czy aktualnie wskazywany jest element listy, czy lista sie skonczyla
	if (rse_list_iterator != rse_list.end())
		return true;
	else
		return false;
}

bool robot_stopped_condition::is_rse_list_last_element(void)
{
	// sprawdza czy aktualnie wskazywany element listy ma nastepnik
	// jesli <> nulla
	if (rse_list_iterator != rse_list.end() ) {
		if ( (++rse_list_iterator) != rse_list.end() ) {
			--rse_list_iterator;
			return false;
		} else {
			--rse_list_iterator;
			return true;
		}; // end if
	}
	return false;
}

void robot_stopped_condition::create_rse_list_head(double robot_position[6], double sensor_reading[6])
{
	// Wstawienie glowy.
	rse_list.push_back(robot_position_digital_scales_reading_element(robot_position, sensor_reading));
	rse_list_iterator = rse_list.begin();
}

void robot_stopped_condition::insert_rse_list_element(double robot_position[6], double sensor_reading[6])
{
	rse_list.push_back(robot_position_digital_scales_reading_element(robot_position, sensor_reading));
	rse_list_iterator = rse_list.begin();
}

int robot_stopped_condition::rse_list_length(void) const
{
	return rse_list.size();
}

/****************** KONIEC: METODY ZWIAZANE Z LISTA RSE ********************/
