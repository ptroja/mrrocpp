// -------------------------------------------------------------------------
//                              task/mp_t_ball.cc
//
// MP task for two robot ball device
//
// -------------------------------------------------------------------------

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"

#include "base/lib/sr/srlib.h"

#include "base/mp/MP_main_error.h"
#include "base/mp/mp_task.h"
#include "application/ball/mp_g_ball.h"
#include "application/ball/mp_t_ball.h"
#include "application/rcsc/ecp_mp_t_rcsc.h"
#include "robot/irp6ot_m/const_irp6ot_m.h"
#include "robot/irp6p_m/const_irp6p_m.h"
#include "subtask/ecp_mp_st_bias_edp_force.h"
#include "generator/ecp/ecp_mp_g_transparent.h"
#include "generator/ecp/ecp_mp_g_newsmooth.h"

#include "robot/conveyor/mp_r_conveyor.h"
#include "robot/irp6ot_m/mp_r_irp6ot_m.h"
#include "robot/irp6p_m/mp_r_irp6p_m.h"
#include "robot/irp6m/mp_r_irp6m.h"
#include "robot/speaker/mp_r_speaker.h"
#include "robot/polycrank/mp_r_polycrank.h"
#include "robot/bird_hand/mp_r_bird_hand.h"
#include "robot/irp6ot_tfg/mp_r_irp6ot_tfg.h"
#include "robot/irp6p_tfg/mp_r_irp6p_tfg.h"
#include "robot/shead/mp_r_shead.h"
#include "robot/spkm/mp_r_spkm.h"
#include "robot/smb/mp_r_smb.h"
#include "robot/sarkofag/mp_r_sarkofag.h"
#include "robot/festival/const_festival.h"
#include "robot/player/const_player.h"

namespace mrrocpp {
namespace mp {
namespace task {

task* return_created_mp_task(lib::configurator &_config)
{
	return new ball(_config);
}

// powolanie robotow w zaleznosci od zawartosci pliku konfiguracyjnego
void ball::create_robots()
{
	ACTIVATE_MP_ROBOT(conveyor);
	ACTIVATE_MP_ROBOT(speaker);
	ACTIVATE_MP_ROBOT(irp6m);
	ACTIVATE_MP_ROBOT(polycrank);
	ACTIVATE_MP_ROBOT(bird_hand);
	ACTIVATE_MP_ROBOT(spkm);
	ACTIVATE_MP_ROBOT(smb);
	ACTIVATE_MP_ROBOT(shead);
	ACTIVATE_MP_ROBOT(irp6ot_tfg);
	ACTIVATE_MP_ROBOT(irp6ot_m);
	ACTIVATE_MP_ROBOT(irp6p_tfg);
	ACTIVATE_MP_ROBOT(irp6p_m);
	ACTIVATE_MP_ROBOT(sarkofag);

	ACTIVATE_MP_DEFAULT_ROBOT(electron);
	ACTIVATE_MP_DEFAULT_ROBOT(speechrecognition);
	ACTIVATE_MP_DEFAULT_ROBOT(festival);

}

ball::ball(lib::configurator &_config) :
	task(_config)
{
}

void ball::configure_edp_force_sensor(bool configure_track, bool configure_postument)
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

void ball::main_task_algorithm(void)
{
	generator::ball mp_h_gen(*this, 10);
	mp_h_gen.robot_m = robot_m;

	//TODO do zamiany na newsmooth
	//set_next_ecps_state(ecp_mp::generator::ECP_GEN_SMOOTH, (int) ecp_mp::task::ABSOLUTE, "src/application/ball/irp6ot_init.trj", 0, 1, lib::irp6ot_m::ROBOT_NAME.c_str());
	//set_next_ecps_state(ecp_mp::generator::ECP_GEN_SMOOTH, (int) ecp_mp::task::ABSOLUTE, "src/application/ball/irp6p_init.trj", 0, 1, lib::irp6p_m::ROBOT_NAME.c_str());

	run_extended_empty_gen_and_wait(2, 2, lib::irp6ot_m::ROBOT_NAME.c_str(), lib::irp6p_m::ROBOT_NAME.c_str(), lib::irp6ot_m::ROBOT_NAME.c_str(), lib::irp6p_m::ROBOT_NAME.c_str());

	sr_ecp_msg->message("New series");
	// wlaczenie generatora do konfiguracji czujnika w EDP w obydwu robotach
	configure_edp_force_sensor(true, true);

	// wlaczenie generatora transparentnego w obu robotach
	set_next_ecps_state(ecp_mp::generator::ECP_GEN_TRANSPARENT, 0, "", 0, 1, lib::irp6ot_m::ROBOT_NAME.c_str());
	set_next_ecps_state(ecp_mp::generator::ECP_GEN_TRANSPARENT, 0, "", 0, 1, lib::irp6p_m::ROBOT_NAME.c_str());

	mp_h_gen.configure(1, 0);
	sr_ecp_msg->message("Track podatny do czasu wcisniecia mp_trigger");
	mp_h_gen.Move();

	send_end_motion_to_ecps(2, lib::irp6ot_m::ROBOT_NAME.c_str(), lib::irp6p_m::ROBOT_NAME.c_str());
}

} // namespace task
} // namespace mp
} // namespace mrrocpp
