//Zadanie ktre realizuje dan trajektori uywajc smooth generatora

#if !defined(_ECP_T_GRAB_CUBE_IRP6OT_H)
#define _ECP_T_GRAB_CUBE_IRP6OT_H

#include "base/ecp/ecp_task.h"
#include "generator/ecp/ecp_g_smooth.h"
#include "robot/irp6ot_m/generator/ecp_g_vis_ib_eih_object_tracker_irp6ot.h"
#include "generator/ecp/force/ecp_g_tff_gripper_approach.h"
#include "ecp_g_vis_ib_eih_wrist_turner_irp6ot.h"
#include "base/lib/com_buf.h"

#include "generator/ecp/force/ecp_g_tff_gripper_approach.h"
#include "generator/ecp/force/ecp_g_bias_edp_force.h"

#include "sensor/fradia/ecp_mp_s_fradia_sensor.h"
#include "sensor/fradia/object_tracker.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot_m {
namespace task {

typedef ecp_mp::sensor::fradia_sensor <lib::empty_t, ecp_mp::sensor::object_tracker_t> fradia_sensor_tracker;

class grab_cube : public common::task::task
{
protected:
	fradia_sensor_tracker *vsp_fradia; //Virtual sensor
	common::generator::smooth* smoothgen2;
	generator::ecp_vis_ib_eih_object_tracker_irp6ot* tracker;
	generator::ecp_vis_ib_eih_wrist_turner_irp6ot* turner;
	common::generator::tff_gripper_approach* gagen;
	common::generator::bias_edp_force* befgen; //calibration of force

public:
	grab_cube(lib::configurator &_config);

	void main_task_algorithm(void);
};

}
} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp

#endif
