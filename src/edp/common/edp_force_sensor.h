// -------------------------------------------------------------------------
//                            vsp.h		dla QNX6.2
//
// Definicje klasy edp_force_sensor
//
// Ostatnia modyfikacja: 2005
// Autor: yoyek
// -------------------------------------------------------------------------

#if !defined(_EDP_FORCE_SENSOR_H)
#define _EDP_FORCE_SENSOR_H

#include <semaphore.h>

#include "lib/mathtr/ForceTrans.h"
#include "lib/sensor.h"				// klasa bazowa sensor
#include "edp/common/edp.h"				// klasa bazowa sensor
#include "edp/common/edp_extension_thread.h"

namespace mrrocpp {
namespace edp {
namespace common {
class irp6s_postument_track_effector;
}
namespace sensor {

/********** klasa czujnikow po stronie EDP **************/
class force: public lib::sensor, public common::edp_extension_thread
{
	protected:
		bool is_reading_ready; // czy jakikolwiek odczyt jest gotowy?

		lib::ForceTrans *gravity_transformation; // klasa likwidujaca wplyw grawitacji na czujnik

	public:
		void create_thread(void);
		static void *thread_start(void* arg);
		void *thread_main_loop(void* arg);

		lib::sr_vsp *sr_msg; //!< komunikacja z SR
		sem_t new_ms; //!< semafor dostepu do nowej wiadomosci dla vsp
		sem_t new_ms_for_edp; //!< semafor dostepu do nowej wiadomosci dla edp
		bool TERMINATE; //!< zakonczenie obydwu watkow
		bool is_sensor_configured; // czy czujnik skonfigurowany?
		bool first_configure_done;
		int set_command_execution_finish();
		int check_for_command_execution_finish();
		virtual void connect_to_hardware(void) = 0;

		double next_force_tool_position[3];
		double next_force_tool_weight;
		double current_force_tool_position[3];
		double current_force_tool_weight;

		bool new_edp_command;
		bool force_sensor_do_configure; // FLAGA ZLECENIA KONFIGURACJI CZUJNIKA
		bool force_sensor_do_first_configure; // pierwsza konfiguracja po synchronizacji lub uruchomieniu
		bool force_sensor_set_tool; // FLAGA ZLECENIA ZMIANY NARZEDZIA

		common::irp6s_postument_track_effector &master;
		force(common::irp6s_postument_track_effector &_master);

		virtual void wait_for_event(void) = 0; // oczekiwanie na zdarzenie
		void set_force_tool(void);

}; // end: class edp_force_sensor


// Zwrocenie stworzonego obiektu - czujnika. Funkcja implementowana w plikach klas dziedziczacych.
force* return_created_edp_force_sensor(common::irp6s_postument_track_effector &_master);

} // namespace sensor
} // namespace edp
} // namespace mrrocpp

#endif
