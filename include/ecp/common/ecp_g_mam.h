// -------------------------------------------------------------------------
// Proces: 	EFFECTOR CONTROL PROCESS (ECP) 
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

#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "ecp/common/ecp_generator.h"
#include "ecp_mp/ecp_mp_s_digital_scales.h"

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
		double robot_position[8];
		double sensor_reading[6];
		mam_element() {};
		mam_element (double _robot_position[8], double _sensor_reading[6]) {
			// Przepisanie pozycji robota.
			memcpy(robot_position, _robot_position, 8*sizeof(double));
			// Przepisanie odczytow z czujnikow.
			memcpy(sensor_reading, _sensor_reading, 6*sizeof(double));
			}; //: mam_element
	}; //: mam_element

// ####################################################################
// #############    KLASA do odtwarzania listy pozycji i odczytywania linialow    ###############
// ####################################################################

class manual_moves_automatic_measures : public common::generator::base {
	private:
		// Ostatnia pozycja.
		double* last_motor_position;
		int UI_fd;
		// Czy pomiar dodany do listy.
		bool measure_added;
		// Liczba osi robota.
	     int axes_number;
		// Lista elementow mam.
	     std::list<mam_element> mam_list;
	     std::list<mam_element>::iterator mam_list_iterator;
		// Metody zwiazane z lista pozycji-odczytow.
		void initiate_mam_list (void);
		void next_mam_list_element (void);
		void get_mam_list_element (mam_element& mam);
		void create_mam_list_head (double* robot_position, double* sensor_reading);
		void insert_mam_list_element (double* robot_position, double* sensor_reading);
		int mam_list_length(void);
		// Sprawdzenie, czy nie pusty.
		bool is_mam_list_element (void);
		bool is_mam_list_last_element ( void );
		// Zapisanie pojedynczego elementu.
		void save_mam_element(ofstream& to_file);

		// Odczyt ostatniej pozycji na liscie.
          void get_mam_list_data (double* robot_position, double* sensor_reading);
		// Pobranie obecnego polozenia robota.
		void get_current_position (double* current_position);
		void get_sensor_reading(ecp_mp::sensor::digital_scales& the_sensor, double* sensor_reading);

	public:
		manual_moves_automatic_measures(common::task::base& _ecp_task, 	int _axes_number);

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
		void save_mam_list(char* filename);
		// Usuniecie wszystkich elementow z listy.
		void flush_mam_list (void);

	}; // end: class manual_moves_automatic_measures_generator

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif
