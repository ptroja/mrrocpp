// -------------------------------------------------------------------------
// Proces: 	EFFECTOR CONTROL PROCESS (ECP)
// Plik:			ecp_trg.h
// System:	QNX/MRROC++  v. 6.3
// Opis:		trajectory_reproduce_generator - deklaracja klasy
// 				generator do odtwarzania trajektorii i odczytywania pozycji z linialow po ruchu robota
// Autor:		tkornuta
// Data:		04.11.2005
// -------------------------------------------------------------------------

#if !defined(_ECP_TRG_H)
#define _ECP_TRG_H

/********************************* INCLUDES *********************************/
#include <list>

#include "common/impconst.h"
#include "common/com_buf.h"

#include "ecp_mp/ecp_mp_s_digital_scales.h"
#include "ecp_mp/ecp_mp_s_force.h"
#include "ecp/common/ecp_teach_in_generator.h"
#include "ecp/common/ECP_main_error.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot {
namespace generator {

// ####################################################################
// #############    KLASA do odtwarzania listy pozycji i odczytywania linialow    ###############
// ####################################################################

class trajectory_reproduce_generator : public common::ecp_teach_in_generator {
	private:
		// Lista pozycji dla danego makrokroku - pozycje posrednie.
	    std::list<common::ecp_taught_in_pose> interpose_list;
	    std::list<common::ecp_taught_in_pose>::iterator interpose_list_iterator;
		// Metody zwiazane z lista pozycjami posrednim.
		void flush_interpose_list (void);
		void initiate_interpose_list (void);
		void next_interpose_list_element (void);
		void get_interpose_list_element (common::ecp_taught_in_pose& tip);
		bool is_interpose_list_element ( void );
		void create_interpose_list_head (POSE_SPECIFICATION ps, double motion_time, double coordinates[6]);
		void insert_interpose_list_element (POSE_SPECIFICATION ps, double motion_time, double coordinates[6]);
		// Sila przy ktorej nalezy przerwac ruch.
		short dangerous_force;
		// ~Aktualny odczyt - czujnik sily.
		double last_force_sensor_reading[6];
		// Pobranie obecnego polozenia robota.
		void get_current_position (double current_position[6]);

	public:
#if !defined(USE_MESSIP_SRR)
		int UI_fd;
#else
		messip_channel_t *UI_fd;
#endif
		trajectory_reproduce_generator(common::task::base& _ecp_task);

		~trajectory_reproduce_generator (void);
		// Przygotowanie trajektorii do wykonania.
		virtual bool first_step ();
		// Wykonanie wlasciwej trajektorii.
		virtual bool next_step ();
		// Przygotowanie generatora do ruchu.
		void prepare_generator_for_motion(void);
		// Wczytanie trajektorii.
		void load_trajectory(char* filename);
		// Stworznie polecenia dla robota -> ruch do pozycji.
		void create_command_for_pose(common::ecp_taught_in_pose& tip);
		void return_sensor_reading(ecp_mp::sensor::force& the_sensor, double sensor_reading[6]);
		void check_force_condition(ecp_mp::sensor::force& the_sensor);
		void set_dangerous_force(void);
		// Zwrocenie danych z tablic.
		void return_current_data(double robot_position[6], double sensor_reading[6]);
		// Obsluga niebezpiecznej sily.
		void dangerous_force_handler(base::ECP_error e);
	}; // end: class trajectory_reproduce_generator

}
} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp

#endif
