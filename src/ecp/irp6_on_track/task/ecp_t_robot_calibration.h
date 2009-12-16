/* file task/ecp_t_robotcalibration.h
 * Class declaration for task of moving the robot - EMARO students
 * author manibaktha
 * date 07.05.2009
 */

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#ifndef _ECP_T_RobotCalibration_H_
#define _ECP_T_RobotCalibration_H_

#include "ecp/common/task/ecp_task.h"

#include "lib/srlib.h"
#include "ecp_mp/task/ecp_mp_t_rcsc.h"
#include "ecp_mp/sensor/ecp_mp_s_schunk.h"

#include "ecp/irp6_on_track/ecp_r_irp6ot.h"

// The generators
#include "ecp/irp6_on_track/generator/ecp_g_robot_calibration.h"
#include "ecp/common/generator/ecp_g_smooth.h"
#include "ecp/common/generator/ecp_g_force.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot {
namespace task {

// Create the class representing the task
class robot_calibration: public common::task::task
{
	char * trajektoria_poczatkowa;
	char * trajektoria_koncowa;

	protected:
		generator::robotcalibgen* generator;
		common::generator::smooth* smooth;

	public:
		robot_calibration(lib::configurator &_config);

		void main_task_algorithm(void);
};

}
} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp

#endif

