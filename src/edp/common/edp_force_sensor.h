// -------------------------------------------------------------------------
//
// Definicje klasy edp_force_sensor
//
// Autor: yoyek
// -------------------------------------------------------------------------

#if !defined(_EDP_FORCE_SENSOR_H)
#define _EDP_FORCE_SENSOR_H

#include <boost/utility.hpp>
#include <boost/thread/mutex.hpp>
#include <Eigen/Core>

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

typedef enum _force_readring_status {
	EDP_FORCE_SENSOR_READING_CORRECT,
	EDP_FORCE_SENSOR_READING_ERROR,
	EDP_FORCE_SENSOR_OVERLOAD
} force_readring_status_t;

typedef struct _force_data
{
	double rez[6]; // by Y pomiar sily

	//! Force reading status
	force_readring_status_t status;
} force_data_t;


/********** klasa czujnikow po stronie EDP **************/
class force: public lib::sensor_interface
{
protected:
	bool is_reading_ready; // czy jakikolwiek odczyt jest gotowy?

	lib::ForceTrans *gravity_transformation; // klasa likwidujaca wplyw grawitacji na czujnik

	common::manip_effector &master;

	virtual void connect_to_hardware(void) = 0;

	struct _from_vsp {
		lib::VSP_REPORT_t vsp_report;
		force_data_t force;
	} from_vsp;

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

	Eigen::Vector3d next_force_tool_position, current_force_tool_position;
	double next_force_tool_weight, current_force_tool_weight;

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
