#include "base/mp/mp_task.h"
#include "mp_t_gen_test.h"

#include "base/lib/mrmath/mrmath.h"

#include "robot/irp6_tfg/dp_tfg.h"
#include "robot/irp6ot_tfg/const_irp6ot_tfg.h"
#include "robot/irp6p_tfg/const_irp6p_tfg.h"
#include "robot/irp6ot_m/const_irp6ot_m.h"
#include "robot/irp6p_m/const_irp6p_m.h"

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"
#include <iostream>
#include <string>
#include <sstream>
#include "base/lib/sr/srlib.h"

#include "ecp_mp_t_gen_test.h"
#include "application/generator_tester/ecp_mp_st_const_vel_gen_test.h"
#include "application/generator_tester/ecp_mp_st_smooth_gen_test.h"

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

//#include "subtask/ecp_mp_st_bias_edp_force.h"
//#include "subtask/ecp_mp_st_tff_nose_run.h"
//#include "generator/ecp/ecp_mp_g_tfg.h"

namespace mrrocpp {
namespace mp {
namespace task {

task* return_created_mp_task(lib::configurator &_config)
{
	return new gen_test(_config);
}

// powolanie robotow w zaleznosci od zawartosci pliku konfiguracyjnego
void gen_test::create_robots()
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

gen_test::gen_test(lib::configurator &_config) :
	task(_config)
{
}

void gen_test::main_task_algorithm(void)
{

	sr_ecp_msg->message("Gen Test (MP) START");

	lib::robot_name_t manipulator_name;
	lib::robot_name_t gripper_name;

	// Track

	if (config.value <int> ("is_irp6ot_m_active", lib::UI_SECTION)) {
		//------------------- CONSTANT VELOCITY GENERATOR -------------------
		//set_next_ecps_state(ecp_mp::task::ECP_ST_CONST_VEL_GEN_TEST, (int) 5, "", 0, 1, lib::irp6ot_m::ROBOT_NAME.c_str());

		//run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots(1, 1, lib::irp6ot_m::ROBOT_NAME.c_str(), lib::irp6ot_m::ROBOT_NAME.c_str());
		//------------------- CONSTANT VELOCITY GENERATOR END -------------------

		//------------------- SMOOTH GENERATOR -------------------
		set_next_ecps_state(ecp_mp::task::ECP_ST_SMOOTH_GEN_TEST, (int) 5, "", 0, 1, lib::irp6ot_m::ROBOT_NAME.c_str());

		run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots(1, 1, lib::irp6ot_m::ROBOT_NAME.c_str(), lib::irp6ot_m::ROBOT_NAME.c_str());
		//------------------- SMOOTH GENERATOR END -------------------
	}

	// Postument

	if (config.value <int> ("is_irp6p_m_active", lib::UI_SECTION)) {
		//------------------- CONSTANT VELOCITY GENERATOR -------------------
		set_next_ecps_state(ecp_mp::task::ECP_ST_CONST_VEL_GEN_TEST, (int) 5, "", 0, 1, lib::irp6p_m::ROBOT_NAME.c_str());

		run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots(1, 1, lib::irp6p_m::ROBOT_NAME.c_str(), lib::irp6p_m::ROBOT_NAME.c_str());
		//------------------- CONSTANT VELOCITY GENERATOR END -------------------

		//------------------- SMOOTH GENERATOR -------------------
		set_next_ecps_state(ecp_mp::task::ECP_ST_SMOOTH_GEN_TEST, (int) 5, "", 0, 1, lib::irp6p_m::ROBOT_NAME.c_str());

		run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots(1, 1, lib::irp6p_m::ROBOT_NAME.c_str(), lib::irp6p_m::ROBOT_NAME.c_str());
		//------------------- SMOOTH GENERATOR END -------------------
	}

	sr_ecp_msg->message("Gen Test END");

}

} // namespace task
} // namespace mp
} // namespace mrrocpp
