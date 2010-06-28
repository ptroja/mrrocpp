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
#include "base/edp/edp.h"				// klasa bazowa sensor
#include "lib/condition_synchroniser.h"

#include "lib/agent/RemoteAgent.h"
#include "lib/agent/RemoteBuffer.h"

namespace mrrocpp {
namespace edp {
namespace common {
class manip_effector;
}
namespace sensor {

typedef enum _force_readring_status
{
	EDP_FORCE_SENSOR_READING_CORRECT, EDP_FORCE_SENSOR_READING_ERROR, EDP_FORCE_SENSOR_OVERLOAD
} force_readring_status_t;

typedef struct _force_data
{
	double rez[6]; // by Y pomiar sily

	//! Force reading status
	force_readring_status_t status;
} force_data_t;

/********** klasa czujnikow po stronie EDP **************/
class force : public lib::sensor_interface
{
private:
	//! Coordinator interested in direct force readings
	RemoteAgent * coordinator;

	//! Coordinator buffer for force readings
	RemoteBuffer<lib::Ft_vector> * remote_buffer;

protected:
	bool is_reading_ready; // czy jakikolwiek odczyt jest gotowy?

	boost::shared_ptr<lib::ForceTrans> gravity_transformation; // klasa likwidujaca wplyw grawitacji na czujnik

	bool is_right_turn_frame;

	common::manip_effector &master;

	virtual void connect_to_hardware(void) = 0;

	struct _from_vsp
	{
		lib::VSP_REPORT_t vsp_report;
		force_data_t force;
	} from_vsp;

	/*!
	 * \brief Info if the force sensor test mode is active.
	 * \todo This should have a 'const' qualifier
	 * It is taken from configuration data.
	 */
	bool test_mode;

	boost::shared_ptr<lib::sr_vsp> sr_msg; //!< komunikacja z SR

	/**
	 * Derived classes are supposed to call this method in get_reading.
	 * The base class will handle transformation to current effector frame.
	 * @param current_ft current force-torque reading from sensor
	 */
	void set_current_ft_reading(const lib::Ft_vector& current_ft);

public:
	void operator()(void);
	boost::mutex mtx;
	lib::condition_synchroniser thread_started;

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

	void set_force_tool();

	//! Default sleep imlpementation
	virtual void wait_for_event();

	//! Default implementation for test mode
	virtual void get_reading();

	//! Common implementation to be called by derived classes
	virtual void configure_sensor();

	//! Default implementation
	virtual void initiate_reading();
}; // end: class edp_force_sensor


// Zwrocenie stworzonego obiektu - czujnika. Funkcja implementowana w plikach klas dziedziczacych.
force* return_created_edp_force_sensor(common::manip_effector &_master);

} // namespace sensor
} // namespace edp
} // namespace mrrocpp

#endif
