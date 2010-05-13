// -------------------------------------------------------------------------
//
// Definicje klasy edp_force_sensor
//
// Autor: yoyek
// -------------------------------------------------------------------------

#if !defined(_EDP_FORCE_SENSOR_H)
#define _EDP_FORCE_SENSOR_H

#include <boost/thread/mutex.hpp>

#include "lib/mrmath/ForceTrans.h"
#include "lib/sensor.h"				// klasa bazowa sensor
#include "edp/common/edp.h"				// klasa bazowa sensor
#include "lib/mis_fun.h"

namespace mrrocpp {
namespace edp {
namespace common {
class manip_effector;
}
namespace sensor {

/********** klasa czujnikow po stronie EDP **************/
class force: public lib::sensor, boost::noncopyable
{
protected:
	bool is_reading_ready; // czy jakikolwiek odczyt jest gotowy?

	lib::ForceTrans *gravity_transformation; // klasa likwidujaca wplyw grawitacji na czujnik

	common::manip_effector &master;

	virtual void connect_to_hardware(void) = 0;

public:
	void operator()(void);
	boost::mutex mtx;
	lib::condition_synchroniser thread_started;

	lib::sr_vsp *sr_msg; //!< komunikacja z SR
	lib::condition_synchroniser edp_vsp_synchroniser;//!< dostep do nowej wiadomosci dla vsp
	lib::condition_synchroniser new_command_synchroniser;//!< dostep do nowej wiadomosci dla vsp
	common::FORCE_ORDER command;

	bool TERMINATE; //!< zakonczenie obydwu watkow
	bool is_sensor_configured; // czy czujnik skonfigurowany?
	void set_command_execution_finish();

	double next_force_tool_position[3];
	double next_force_tool_weight;
	double current_force_tool_position[3];
	double current_force_tool_weight;

	bool new_edp_command;

	force(common::manip_effector &_master);

	virtual ~force();

	virtual void wait_for_event(void) = 0; // oczekiwanie na zdarzenie
	void set_force_tool(void);
}; // end: class edp_force_sensor


// Zwrocenie stworzonego obiektu - czujnika. Funkcja implementowana w plikach klas dziedziczacych.
force* return_created_edp_force_sensor(common::manip_effector &_master);

} // namespace sensor
} // namespace edp
} // namespace mrrocpp

#endif
