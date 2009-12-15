// -------------------------------------------------------------------------
// Proces: 	EFFECTOR CONTROL PROCESS (lib::ECP)
// Plik:			ecp_rsc.h
// System:	QNX/MRROC++  v. 6.3
// Opis:		robot_stopped_condition - deklaracja klasy
// 				condition - sprawdzanie, czy robot sie zatrzymal
// Autor:		tkornuta
// Data:		04.11.2005
// -------------------------------------------------------------------------

#if !defined(_ECP_RSC_H)
#define _ECP_RSC_H

/********************************* INCLUDES *********************************/
#include <string.h>
#include <list>

#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "ecp/common/ecp_generator.h"
#include "ecp/common/ECP_main_error.h"
#include "ecp_mp/sensor/ecp_mp_s_digital_scales.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot {
namespace generator {

// ####################################################################
// ################ KLASA Robot Position Digital Scales Reading  Element ##################
// #############  do przechowywania par (pozycja robota) - (odczyty linialow)  ###############
// ####################################################################

class robot_position_digital_scales_reading_element{
	public:
		double robot_position[6];
		double sensor_reading[6];
		robot_position_digital_scales_reading_element() {};
		robot_position_digital_scales_reading_element (double _robot_position[6], double _sensor_reading[6]) {
			// przepisanie pozycji robota
			memcpy(robot_position, _robot_position, MAX_SERVOS_NR*sizeof(double));
			// przepisanie odczytow z czujnikow
			memcpy(sensor_reading, _sensor_reading, MAX_SERVOS_NR*sizeof(double));
			}; // end: ecp_taught_in_pose::ecp_taught_in_pose
	}; // end:class trg_readings

// ####################################################################
// #################   KLASA do sprawdzania, czy robot stoi w miejscu    ##################
// ####################################################################

class robot_stopped_condition : public common::generator::generator {
	private:
		// Lista elementow RSE.
	    std::list<robot_position_digital_scales_reading_element> rse_list;
	    std::list<robot_position_digital_scales_reading_element>::iterator rse_list_iterator;
		// Metody zwiazane z lista pozycji-odczytow.
		void flush_rse_list (void);
		void initiate_rse_list (void);
		void next_rse_list_element (void);
		void get_rse_list_element (robot_position_digital_scales_reading_element& rse);
		void create_rse_list_head (double robot_position[6], double sensor_reading[6]);
		void insert_rse_list_element (double robot_position[6], double sensor_reading[6]);
		int rse_list_length(void) const;
		// Sprawdzenie, czy nie pusty.
		bool is_rse_list_element (void);
		bool is_rse_list_last_element ( void );
		// Ostatni odczyt czujnika sily.
		double last_force_sensor_reading[6];
	public:
#if !defined(USE_MESSIP_SRR)
		int UI_fd;
#else
		messip_channel_t *UI_fd;
#endif
		robot_stopped_condition(common::task::task& _ecp_task);
		~robot_stopped_condition(void);
		// Przygotowanie warunku do ruchu.
		void prepare_condition_for_motion(void);
		// Dodanie nowego elementu do listy.
		void add_rse_element(ecp_mp::sensor::digital_scales& the_sensor);
		// Odczyt ostatniej pozycji na liscie.
		void get_rse_list_data (double robot_position[6], double sensor_reading[6]);
		// Zapisanie wynikow.
		void save_rse_list(char* filename);
		// Wyslanie polecenia odswiezenia okna.
		void refresh_window(void);
		// Sprawdzenie warunku.

		  virtual bool first_step ();
		  virtual bool next_step ();
		// Pobranie pozycji robota.
		void get_current_position(double current_position[6]);
	}; // end: class robot_stopped_condition

}
} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp

#endif
