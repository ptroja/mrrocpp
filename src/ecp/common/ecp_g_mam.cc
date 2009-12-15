// -------------------------------------------------------------------------
// Proces: 	EFFECTOR CONTROL PROCESS (lib::ECP)
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

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/srlib.h"

#include "ecp/common/ecp_g_mam.h"
#include "ecp_mp/sensor/ecp_mp_s_digital_scales.h"
#include "ecp/common/ECP_main_error.h"

#if defined(USE_MESSIP_SRR)
#include "lib/messip/messip_dataport.h"
#endif

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

/*********************** METODY ZWIAZANE Z LISTA MAM ************************/

void manual_moves_automatic_measures::flush_mam_list ( void ) {
	// Jezeli sa jakies elementy - usuniecie.
	mam_list.clear();

	// Wyzerowanie ostatniego polozenia.
	for(int i =0; i<axes_number; i++)
		last_motor_position[i] = 0;
	// Pomiar nie zostal dodany do listy.
	measure_added = false;
}

void manual_moves_automatic_measures::initiate_mam_list(void) {
	mam_list_iterator = mam_list.begin();
}

void manual_moves_automatic_measures::next_mam_list_element (void) {
	// Przejscie na nastepny element.
	mam_list_iterator++;
}

void manual_moves_automatic_measures::get_mam_list_element (mam_element& mam){
	// Przepisanie pozycji robota.
	memcpy(mam.robot_position, mam_list_iterator->robot_position, axes_number*sizeof(double));
	// Przepisanie odczytow z czujnikow.
	memcpy(mam.sensor_reading, mam_list_iterator->sensor_reading, 6*sizeof(double));
}

void manual_moves_automatic_measures::get_mam_list_data (double* robot_position, double* sensor_reading){
	// Przepisanie pozycji robota.
	memcpy(robot_position, mam_list_iterator->robot_position, axes_number*sizeof(double));
	// Przepisanie odczytow z czujnikow.
	memcpy(sensor_reading, mam_list_iterator->sensor_reading, 6*sizeof(double));
}

bool manual_moves_automatic_measures::is_mam_list_element ( void ) {
	// sprawdza czy aktualnie wskazywany jest element listy, czy lista sie skonczyla
	return (mam_list_iterator != mam_list.end());
}

bool manual_moves_automatic_measures::is_mam_list_last_element ( void ) {
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
	}
	return false;
}

void manual_moves_automatic_measures::create_mam_list_head (const double* robot_position, const double* sensor_reading) {
	// Wstawienie glowy.
	mam_list.push_back(mam_element (robot_position, sensor_reading));
	mam_list_iterator = mam_list.begin();
}

void manual_moves_automatic_measures::insert_mam_list_element (const double* robot_position, const double* sensor_reading) {
	// Wlasciwe wstawienie elementu.
	mam_list.push_back(mam_element (robot_position, sensor_reading));
	mam_list_iterator++;
}

int manual_moves_automatic_measures::mam_list_length(void) const
{
	return mam_list.size();
}

/****************** KONIEC: METODY ZWIAZANE Z LISTA MAM ********************/


/*****************************  KONSTRUKTOR *********************************/
manual_moves_automatic_measures::manual_moves_automatic_measures(common::task::task& _ecp_task, int _axes_number) :
	generator(_ecp_task),
	UI_fd(_ecp_task.UI_fd),
	measure_added(false),	// Pomiar nie zostal dodany do listy.
	axes_number(_axes_number)
{
	// Przydzielenie pamieci na pozycje z ostatniego ruchu.
	last_motor_position = new double [axes_number];

	// Wyzerowanie ostatniego polozenia.
	for (int i =0; i<axes_number; i++)
		last_motor_position[i] = 0;
}

/******************************  DESTRUKTOR **********************************/
manual_moves_automatic_measures::~manual_moves_automatic_measures(void)
{
	// Zwolnienie pamieci.
	delete(last_motor_position);
}


/******************************** FIRST STEP ***********************************/
bool manual_moves_automatic_measures::first_step (){
    // Przygotowanie rozkazu dla EDP.
    the_robot->ecp_command.instruction.instruction_type = lib::GET;
    the_robot->ecp_command.instruction.get_type = ARM_DV; // ARM
    // Sprawdzenie rodzaju ramienia.
    the_robot->ecp_command.instruction.get_arm_type = lib::MOTOR;

     // Mozna wykonac ruch - odebrac polozenie
    return true;
}

/******************************** NEXT STEP ***********************************/
bool manual_moves_automatic_measures::next_step (){
    // Roznica miedzy polozeniami.
    double eps = 1e-2;

	// Czujnik - linialy.
    ecp_mp::sensor::digital_scales* dss = (ecp_mp::sensor::digital_scales*)((sensor_m.begin())->second);
	// Porownanie ostatniego polozenia z obecnym.
	bool position_changed = false;
	for(int i =0; i<axes_number; i++)
		if (the_robot->reply_package.arm.pf_def.arm_coordinates[i] - last_motor_position[i] > eps)
		{
			// Polozenia rozne.
			position_changed = true;
			break;
		}
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
		}
	}
    // Odswiezenie okna.
    refresh_window(*dss);
     // Koniec ruchu.
    return false;
}

/************************* GET CURRENT POSITION *****************************/
void manual_moves_automatic_measures::get_current_position(double* current_position){
    // Przepisanie obecnego polozenia robota do bufora w zaleznosci od rodzaju wspolrzednych.
    memcpy(current_position, the_robot->reply_package.arm.pf_def.arm_coordinates, axes_number*sizeof(double));
}

/*********************** RETURN SENSOR READING ***************************/
void manual_moves_automatic_measures::get_sensor_reading(ecp_mp::sensor::digital_scales& the_sensor, double* sensor_reading){
    // Przepisanie pozycji z bufora.
    memcpy(sensor_reading, the_sensor.image.sensor_union.ds.readings, 6*sizeof(double));
}

/**************************** REFRESH WINDOW *******************************/
void manual_moves_automatic_measures::refresh_window (ecp_mp::sensor::digital_scales& the_sensor)
{
	// Wiadomosc wysylana do UI.
	lib::ECP_message ecp_ui_msg;

	// Polecenie odswiezenia okna.
	ecp_ui_msg.ecp_message = lib::MAM_REFRESH_WINDOW;
	// Przepisanie polozenia oraz odczytow z linialow do wiadomosci.
	get_current_position(ecp_ui_msg.MAM.robot_position);
	get_sensor_reading(the_sensor, ecp_ui_msg.MAM.sensor_reading);
	ecp_ui_msg.MAM.measure_number = mam_list_length();

	// Wyslanie polecenia do UI -> Polecenie odswiezenia okna.
#if !defined(USE_MESSIP_SRR)
	ecp_ui_msg.hdr.type=0;
	if (MsgSend(UI_fd, &ecp_ui_msg, sizeof(ecp_ui_msg), NULL, 0) < 0)
#else
	if (messip::port_send_sync(UI_fd, 0, 0, ecp_ui_msg) < 0)
#endif
	{
		 perror("ECP trajectory_reproduce_thread(): Send() to UI failed");
		 sr_ecp_msg.message (lib::SYSTEM_ERROR, errno, "ECP: Send() to UI failed");
	}
}

/***************************** ADD MAM ELEMENT *******************************/
void manual_moves_automatic_measures::add_mam_element(ecp_mp::sensor::digital_scales& the_sensor){
	// Dodanie elementu do listy
	if (mam_list.empty()){
		// Jesli glowa pusta.
		create_mam_list_head(
			the_robot->reply_package.arm.pf_def.arm_coordinates,
			the_sensor.image.sensor_union.ds.readings);
	} else {
		// Jesli nastepny element.
		insert_mam_list_element(
			the_robot->reply_package.arm.pf_def.arm_coordinates,
			the_sensor.image.sensor_union.ds.readings);
	}
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
}

/**************************** SAVE MAM ELEMENTS ******************************/
void manual_moves_automatic_measures::save_mam_element(std::ofstream& to_file)
{
	// Element listy.
	mam_element mam;
	// Pobranie elementu.
	get_mam_list_element (mam);
	// Zapis polozenia robota.
	for(int i=0; i<axes_number; i++)
		to_file << mam.robot_position[i] << ' ';
	// Zapis odczytow z czujnika
	for(int i=0; i<6; i++)
		to_file << mam.sensor_reading[i] << ' ';
	// Nastepna linia.
	to_file << '\n';
}//: save_mam_element

void manual_moves_automatic_measures::save_mam_list(const std::string & filename)
{
	// Sprawdzenie, czy lista nie jest pusta.
	if (mam_list_length() == 0) {
		sr_ecp_msg.message("MAM list empty.");
		return;
	}
	try {
		// Otworzenie pliku.
        std::ofstream to_file(filename.c_str());
		if (!to_file.good())
		throw ECP_main_error(lib::FATAL_ERROR, SAVE_FILE_ERROR);
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
		// Komentarz - zapisanie pliku.
		sr_ecp_msg.message ("Measures saved properly to file");
	}
	catch(ECP_main_error e) {
		// Wylapanie i oblsuga bledow.
		sr_ecp_msg.message (e.error_class, e.error_no);
	}
}

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp
