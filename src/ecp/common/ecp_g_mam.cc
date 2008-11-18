// -------------------------------------------------------------------------
// Proces: 	EFFECTOR CONTROL PROCESS (ECP) 
// Plik:			ecp_gen_mam.h
// System:	QNX/MRROC++  v. 6.3
// Opis:		manual_moves_automatic_measures_generator - definicja metod klasy
// 				generator do zbierania danych pomiarowych z linialow oraz pozycji robota
// Autor:		tkornuta
// Data:		22.03.2006
// -------------------------------------------------------------------------

/********************************* INCLUDES *********************************/

#include <stdio.h>
#include <errno.h>
#include <fstream>
#if defined(__QNXNTO__)
#include <sys/neutrino.h>
#endif /* __QNXNTO__ */

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"

#include "lib/srlib.h"

#include "ecp/common/ecp_g_mam.h"
#include "ecp_mp/ecp_mp_s_digital_scales.h"
#include "ecp/common/ECP_main_error.h"

/*********************** METODY ZWIAZANE Z LISTA MAM ************************/

void manual_moves_automatic_measures_generator::flush_mam_list ( void ) {
	// Jezeli sa jakies elementy - usuniecie.
	mam_list.clear();

	// Wyzerowanie ostatniego polozenia.
	for(int i =0; i<axes_number; i++)
		last_motor_position[i] = 0;
	// Pomiar nie zostal dodany do listy.
	measure_added = false;
	}; // end: flush_mam_list

void manual_moves_automatic_measures_generator::initiate_mam_list(void) {
	mam_list_iterator = mam_list.begin();
	};

void manual_moves_automatic_measures_generator::next_mam_list_element (void) {
	// Przejscie na nastepny element.
	mam_list_iterator++;
	};

void manual_moves_automatic_measures_generator::get_mam_list_element (mam_element& mam){
	// Przepisanie pozycji robota.
	memcpy(mam.robot_position, mam_list_iterator->robot_position, axes_number*sizeof(double));
	// Przepisanie odczytow z czujnikow.
	memcpy(mam.sensor_reading, mam_list_iterator->sensor_reading, 6*sizeof(double));
	};

void manual_moves_automatic_measures_generator::get_mam_list_data (double* robot_position, double* sensor_reading){
	// Przepisanie pozycji robota.
	memcpy(robot_position, mam_list_iterator->robot_position, axes_number*sizeof(double));
	// Przepisanie odczytow z czujnikow.
	memcpy(sensor_reading, mam_list_iterator->sensor_reading, 6*sizeof(double));
};
			
bool manual_moves_automatic_measures_generator::is_mam_list_element ( void ) {
	// sprawdza czy aktualnie wskazywany jest element listy, czy lista sie skonczyla
	if (mam_list_iterator != mam_list.end())
		return true;
	else
		return false;
};
	
bool manual_moves_automatic_measures_generator::is_mam_list_last_element ( void ) {
	// sprawdza czy aktualnie wskazywany element listy ma nastepnik
	// jesli <> nulla
	if (mam_list_iterator != mam_list.end()){
		// jesli nastepnik <> nulla
		if ((++mam_list_iterator) != mam_list.end())
		{
			mam_list_iterator--;
			return false;
		}
		else
		{
			mam_list_iterator--;
			return true;
		}
	}; // end if
	return false;
};

void manual_moves_automatic_measures_generator::create_mam_list_head (double* robot_position, double* sensor_reading) {
	// Wstawienie glowy.
	mam_list.push_back(mam_element (robot_position, sensor_reading));
	mam_list_iterator = mam_list.begin();
	};
			
void manual_moves_automatic_measures_generator::insert_mam_list_element (double* robot_position, double* sensor_reading) {
	// Wlasciwe wstawienie elementu.
	mam_list.push_back(mam_element (robot_position, sensor_reading));
	mam_list_iterator++;
	};

int manual_moves_automatic_measures_generator::mam_list_length(void) {
	return mam_list.size();
};

/****************** KONIEC: METODY ZWIAZANE Z LISTA MAM ********************/


/*****************************  KONSTRUKTOR *********************************/
manual_moves_automatic_measures_generator::manual_moves_automatic_measures_generator(ecp_task& _ecp_task, int _axes_number) :
	ecp_generator(_ecp_task)
{
	UI_fd = _ecp_task.UI_fd;
	// Ustawienie elementow list na NULL.
	mam_list.clear();
	// Przepisanie liczby osi robota.
	axes_number = _axes_number;
	// Przydzielenie pamieci na pozycje z ostatniego ruchu.
	last_motor_position = new double [axes_number];
	// Wyzerowanie ostatniego polozenia.
	for (int i =0; i<axes_number; i++)
		last_motor_position[i] = 0;
	// Pomiar nie zostal dodany do listy.
	measure_added = false;
}

/******************************  DESTRUKTOR **********************************/
manual_moves_automatic_measures_generator::~manual_moves_automatic_measures_generator(void)
{
	// Usuniecie elementow z listy MAM.
	flush_mam_list();
	// Zwolnienie pamieci.
	delete(last_motor_position);
}


/******************************** FIRST STEP ***********************************/
bool manual_moves_automatic_measures_generator::first_step (){
    // Przygotowanie rozkazu dla EDP.
    the_robot->EDP_data.instruction_type = GET;
    the_robot->EDP_data.get_type = ARM_DV; // ARM
    // Sprawdzenie rodzaju ramienia.
    the_robot->EDP_data.get_arm_type = MOTOR;
 
     // Mozna wykonac ruch - odebrac polozenie
    return true;
    }; // end: first_step


/******************************** NEXT STEP ***********************************/
bool manual_moves_automatic_measures_generator::next_step (){
    // Roznica miedzy polozeniami.
    double eps = 1e-2;

	// Czujnik - linialy.
	ecp_mp_digital_scales_sensor* dss = (ecp_mp_digital_scales_sensor*)((sensor_m.begin())->second);
	// Porownanie ostatniego polozenia z obecnym.
	bool position_changed = false;
	for(int i =0; i<axes_number; i++)
		if (the_robot->EDP_data.current_motor_arm_coordinates[i] - last_motor_position[i] > eps)
		{
			// Polozenia rozne.
			position_changed = true;
			break;
		};
	// Jezeli polozenia rozne - zapamietanie obecnego.
	if (position_changed)
	{
		// Zapamietanie nowego polozenia.
		get_current_position(last_motor_position);
		// Pomiar nie zostal dodany do listy.
		measure_added = false;
	}
	else
	{
		if (!measure_added)
		{
			// Pomiar jest ok oraz wczesniej nie zostal dodany do listy - dodajemy.
			add_mam_element(*dss);
			measure_added = true;
			printf("\a\n");
		};
	};
    // Odswiezenie okna.
    refresh_window(*dss);
     // Koniec ruchu.
    return false;
    }; // end: next_step


/************************* GET CURRENT POSITION *****************************/
void manual_moves_automatic_measures_generator::get_current_position(double* current_position){
    // Przepisanie obecnego polozenia robota do bufora w zaleznosci od rodzaju wspolrzednych.
    memcpy(current_position, the_robot->EDP_data.current_motor_arm_coordinates, axes_number*sizeof(double));
    }; // end: get_current_position

/*********************** RETURN SENSOR READING ***************************/
void manual_moves_automatic_measures_generator::get_sensor_reading(ecp_mp_digital_scales_sensor& the_sensor, double* sensor_reading){
    // Przepisanie pozycji z bufora.
    memcpy(sensor_reading, the_sensor.image.sensor_union.ds.readings, 6*sizeof(double));
    }; // end: return_sensor_reading



/**************************** REFRESH WINDOW *******************************/
void manual_moves_automatic_measures_generator::refresh_window
	(ecp_mp_digital_scales_sensor& the_sensor){
	// Wiadomosc wysylana do UI.
	ECP_message ecp_ui_msg;
	// Odswiezenie okna.
	ecp_ui_msg.hdr.type=0;
	// Polecenie odswiezenia okna.
	ecp_ui_msg.ecp_message = MAM_REFRESH_WINDOW;
	// Przepisanie polozenia oraz odczytow z linialow do wiadomosci.
	get_current_position(ecp_ui_msg.MAM.robot_position);
	get_sensor_reading(the_sensor, ecp_ui_msg.MAM.sensor_reading);
	ecp_ui_msg.MAM.measure_number = mam_list_length();
/*
		for(int j=0; j<8; j++)
			printf("%f\t",ecp_ui_msg.MAM.robot_position[j]);
		printf("\n");
*/
	// Wyslanie polecenia do UI -> Polecenie odswiezenia okna.
	if (MsgSend(UI_fd, &ecp_ui_msg,  sizeof(ECP_message),  NULL, 0) < 0){
		 perror("ECP trajectory_reproduce_thread(): Send() to UI failed");
		 sr_ecp_msg.message (SYSTEM_ERROR, errno, "ECP: Send() to UI failed");
		};
	}; //: refresh_window


/***************************** ADD MAM ELEMENT *******************************/
void manual_moves_automatic_measures_generator::add_mam_element(ecp_mp_digital_scales_sensor& the_sensor){
	// Dodanie elementu do listy
	if (mam_list.empty()){
		// Jesli glowa pusta.
		create_mam_list_head(
			the_robot->EDP_data.current_motor_arm_coordinates,
			the_sensor.image.sensor_union.ds.readings);
	}else{
		// Jesli nastepny element.
		insert_mam_list_element(
			the_robot->EDP_data.current_motor_arm_coordinates,
			the_sensor.image.sensor_union.ds.readings);
		}; // end else
	// Wyswietlenie dodanego elementu.
	mam_element mam;
	get_mam_list_element(mam);

/*	printf("Robot :: ");
	for(i=0; i<axes_number; i++)
		printf("%f\t", mam.robot_position[i]);
	printf("\nCzujnik :: ");
	for(i=0; i<6; i++)
		printf("%f\t", mam.sensor_reading[i]);
	printf("\n");*/
	};//: add_mam_element

/**************************** SAVE MAM ELEMENTS ******************************/
void manual_moves_automatic_measures_generator::save_mam_element(ofstream& to_file)
{
	int i;
	// Element listy.
	mam_element mam;
	// Pobranie elementu.
	get_mam_list_element (mam);
	// Zapis polozenia robota.
	for(i=0; i<axes_number; i++)
		to_file << mam.robot_position[i] << ' ';
	// Zapis odczytow z czujnika
	for(i=0; i<6; i++)
		to_file << mam.sensor_reading[i] << ' ';
	// Nastepna linia.
	to_file << '\n';
}//: save_mam_element

void manual_moves_automatic_measures_generator::save_mam_list(char* filename)
{
	// Sprawdzenie, czy lista nie jest pusta.
	if (mam_list_length() == 0) {
		sr_ecp_msg.message("MAM list empty.");
		return;
	}
	try {
		// Otworzenie pliku.
		ofstream to_file(filename);
		if (!to_file)
		throw ECP_main_error(FATAL_ERROR, SAVE_FILE_ERROR);
		// Przejscie na poczatek listy.
		initiate_mam_list();
		// Zapisywanie kolejnych elementow.
		while (!is_mam_list_last_element()) {
			save_mam_element(to_file);
			// Nastepna pozycja.
			next_mam_list_element();
		}
		// zapisanie ostatniego elementu
		save_mam_element(to_file);
		// Zamkniecie pliku.
		to_file.close();
		// Komentarz - zapisanie pliku.
		sr_ecp_msg.message ("Measures saved properly to file");
	}
	catch(ECP_main_error e) {
		// Wylapanie i oblsuga bledow.
		sr_ecp_msg.message (e.error_class, e.error_no);
	}
}
