// -------------------------------------------------------------------------
//                              task/mp_t_haptic.cc
//
// MP task for two robot haptic device
//
// -------------------------------------------------------------------------

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"

#include "base/lib/sr/srlib.h"

#include "base/mp/MP_main_error.h"
#include "base/mp/mp_robot.h"
#include "base/mp/mp_task.h"
#include "application/haptic/mp_g_haptic.h"

#include "application/haptic/mp_t_haptic.h"
#include "application/rcsc/ecp_mp_t_rcsc.h"
#include "subtask/ecp_mp_st_bias_edp_force.h"
#include "generator/ecp/ecp_mp_g_transparent.h"

#include "robot/irp6ot_m/mp_r_irp6ot_m.h"
#include "robot/irp6p_m/mp_r_irp6p_m.h"

namespace mrrocpp {
namespace mp {
namespace task {

task* return_created_mp_task(lib::configurator &_config)
{
	return new haptic(_config);
}

haptic::haptic(lib::configurator &_config) :
	task(_config)
{
}

// powolanie robotow w zaleznosci od zawartosci pliku konfiguracyjnego
void haptic::create_robots()
{

	ACTIVATE_MP_ROBOT(irp6ot_m);
	ACTIVATE_MP_ROBOT(irp6p_m);

}

void haptic::configure_edp_force_sensor(bool configure_track, bool configure_postument)
{
	if (configure_track) {
		set_next_ecps_state(ecp_mp::sub_task::ECP_ST_BIAS_EDP_FORCE, 0, "", 0, 1, lib::irp6ot_m::ROBOT_NAME.c_str());
	}

	if (configure_postument) {
		set_next_ecps_state(ecp_mp::sub_task::ECP_ST_BIAS_EDP_FORCE, 0, "", 0, 1, lib::irp6p_m::ROBOT_NAME.c_str());
	}

	if ((configure_track) && (!configure_postument)) {
		run_extended_empty_gen_and_wait(1, 1, lib::irp6ot_m::ROBOT_NAME.c_str(), lib::irp6ot_m::ROBOT_NAME.c_str());
	} else if ((!configure_track) && (configure_postument)) {
		run_extended_empty_gen_and_wait(1, 1, lib::irp6p_m::ROBOT_NAME.c_str(), lib::irp6p_m::ROBOT_NAME.c_str());
	} else if ((configure_track) && (configure_postument)) {
		run_extended_empty_gen_and_wait(2, 2, lib::irp6ot_m::ROBOT_NAME.c_str(), lib::irp6p_m::ROBOT_NAME.c_str(), lib::irp6ot_m::ROBOT_NAME.c_str(), lib::irp6p_m::ROBOT_NAME.c_str());
	}
}

void haptic::main_task_algorithm(void)
{
	generator::haptic mp_h_gen(*this, 10);
	mp_h_gen.robot_m = robot_m;

	sr_ecp_msg->message("New series");
	//pierwsza konfiguracja czujnikow
	// wlaczenie generatora do konfiguracji czujnika w EDP w obydwu robotach
	configure_edp_force_sensor(true, true);

	// wlaczenie generatora transparentnego w obu robotach
	set_next_ecps_state(ecp_mp::generator::ECP_GEN_TRANSPARENT, (int) 0, "", 0, 1, lib::irp6ot_m::ROBOT_NAME.c_str());
	set_next_ecps_state(ecp_mp::generator::ECP_GEN_TRANSPARENT, (int) 0, "", 0, 1, lib::irp6p_m::ROBOT_NAME.c_str());

	// mp_h_gen.sensor_m = sensor_m;
	mp_h_gen.configure(1, 0);
	sr_ecp_msg->message("Track podatny do czasu wcisniecia mp_trigger");
	mp_h_gen.Move();

	send_end_motion_to_ecps(2, lib::irp6ot_m::ROBOT_NAME.c_str(), lib::irp6p_m::ROBOT_NAME.c_str());
}

} // namespace task
} // namespace mp
} // namespace mrrocpp
