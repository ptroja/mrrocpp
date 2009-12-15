// -------------------------------------------------------------------------
// Proces: 	EFFECTOR CONTROL PROCESS (lib::ECP)
// Plik:			ecp_gen_mam.h
// System:	QNX/MRROC++  v. 6.3
// Opis:		manual_moves_automatic_measures_generator - deklaracja klasy
// 				generator do zbierania danych pomiarowych z linialow oraz pozycji robota
// Autor:		tkornuta
// Data:		16.03.2006
// -------------------------------------------------------------------------

#if !defined(_ECP_GEN_MAM_H)
#define _ECP_GEN_MAM_H

/********************************* INCLUDES *********************************/
#include <string.h>
#include <list>
#include <fstream>

#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "ecp/common/generator/ecp_generator.h"
#include "ecp_mp/sensor/ecp_mp_s_digital_scales.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

// ####################################################################
// #########################  KLASA MAM Element  #############################
// #############  do przechowywania par (pozycja robota) - (odczyty linialow)  ###############
// ####################################################################

class mam_element{
	public:
		double robot_position[MAX_SERVOS_NR];
		double sensor_reading[6];
		mam_element() {};
		mam_element (const double _robot_position[MAX_SERVOS_NR], const double _sensor_reading[6]) {
			// Przepisanie pozycji robota.
			memcpy(robot_position, _robot_position, MAX_SERVOS_NR*sizeof(double));
			// Przepisanie odczytow z czujnikow.
			memcpy(sensor_reading, _sensor_reading, 6*sizeof(double));
			}; //: mam_element
	}; //: mam_element

// ####################################################################
// #############    KLASA do odtwarzania listy pozycji i odczytywania linialow    ###############
// ####################################################################

class manual_moves_automatic_measures : public common::generator::generator {
private:
	// Ostatnia pozycja.
	double* last_motor_position;
#if !defined(USE_MESSIP_SRR)
	const int UI_fd;
#else
	messip_channel_t *UI_fd;
#endif
	// Czy pomiar dodany do listy.
	bool measure_added;
	// Liczba osi robota.
	const int axes_number;
	// Lista elementow mam.
	std::list<mam_element> mam_list;
	std::list<mam_element>::iterator mam_list_iterator;
	// Metody zwiazane z lista pozycji-odczytow.
	void initiate_mam_list (void);
	void next_mam_list_element (void);
	void get_mam_list_element (mam_element& mam);
	void create_mam_list_head (const double* robot_position, const double* sensor_reading);
	void insert_mam_list_element (const double* robot_position, const double* sensor_reading);
	int mam_list_length(void) const;
	// Sprawdzenie, czy nie pusty.
	bool is_mam_list_element (void);
	bool is_mam_list_last_element ( void );
	// Zapisanie pojedynczego elementu.
	void save_mam_element(std::ofstream& to_file);

	// Odczyt ostatniej pozycji na liscie.
	void get_mam_list_data (double* robot_position, double* sensor_reading);
	// Pobranie obecnego polozenia robota.
	void get_current_position (double* current_position);
	void get_sensor_reading(ecp_mp::sensor::digital_scales& the_sensor, double* sensor_reading);

public:
	manual_moves_automatic_measures(common::task::task& _ecp_task, int _axes_number);

	~manual_moves_automatic_measures (void);
	// Przygotowanie trajektorii do wykonania.
	virtual bool first_step ();
	// Wykonanie wlasciwej trajektorii.
	virtual bool next_step ();

	// Wyslanie polecenia odswiezenia okna.
	void refresh_window(ecp_mp::sensor::digital_scales& the_sensor);

	// Dodanie nowego elementu do listy.
	void add_mam_element(ecp_mp::sensor::digital_scales& the_sensor);
	// Zapisanie wynikow do pliku.
	void save_mam_list(const std::string & filename);
	// Usuniecie wszystkich elementow z listy.
	void flush_mam_list (void);

}; // end: class manual_moves_automatic_measures_generator

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif
