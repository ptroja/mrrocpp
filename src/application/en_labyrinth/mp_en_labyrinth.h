#ifndef MP_EN_LABYRINTH_H
#define MP_EN_LABYRINTH_H

#include "base/mp/mp_task.h"
#include "robot/irp6ot_m/mp_r_irp6ot_m.h"
#include "robot/irp6p_m/mp_r_irp6p_m.h"
#include "robot/irp6ot_tfg/mp_r_irp6ot_tfg.h"
#include "robot/irp6p_tfg/mp_r_irp6p_tfg.h"
#include "sensor/discode/discode_sensor.h"
#include <boost/shared_ptr.hpp>

#define UP 0
#define RIGHT 1
#define DOWN 2
#define LEFT 3

using mrrocpp::ecp_mp::sensor::discode::discode_sensor;


namespace mrrocpp {
namespace mp {
namespace task {

class mp_en_labyrinth : public task
{

public:

	mp_en_labyrinth(lib::configurator &_config);
	void main_task_algorithm(void);
	void create_robots(void);
	void runWaitFunction(int time);

private:
	boost::shared_ptr<discode_sensor> discode;

	bool ERROR;

}; // end : class mp_task_fsautomat

} // namespace task
} // namespace mp
} // namespace mrrocpp

#endif
