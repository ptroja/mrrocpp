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
#include "base/lib/srlib.h"

#include "ecp_mp_t_gen_test.h"
#include "application/generator_tester/ecp_mp_st_const_vel_gen_test.h"
#include "application/generator_tester/ecp_mp_st_smooth_gen_test.h"

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

gen_test::gen_test(lib::configurator &_config) :
	task(_config)
{
}

void gen_test::main_task_algorithm(void)
{

	sr_ecp_msg->message("Gen Test (MP) START");

	lib::robot_name_t manipulator_name;
	lib::robot_name_t gripper_name;

	// ROBOT IRP6_ON_TRACK_MANIPULATOR
	if (config.value <int> ("is_irp6ot_m_active", UI_SECTION)) {
		manipulator_name = lib::irp6ot_m::ROBOT_IRP6OT_M;
		if (config.value <int> ("is_irp6ot_tfg_active", UI_SECTION)) {
			gripper_name = lib::irp6ot_tfg::ROBOT_IRP6OT_TFG;
		} else {
			// TODO: throw
		}
		// ROBOT IRP6_POSTUMENT_MANIPULATOR
	} else if (config.value <int> ("is_irp6p_m_active", UI_SECTION)) {
		manipulator_name = lib::irp6p_m::ROBOT_IRP6P_M;
		if (config.value <int> ("is_irp6p_tfg_active", UI_SECTION)) {
			gripper_name = lib::ROBOT_IRP6P_TFG;
		} else {
			// TODO: throw
		}
	} else {
		// TODO: throw
	}

	//------------------- CONSTANT VELOCITY GENERATOR -------------------
	set_next_ecps_state(ecp_mp::task::ECP_ST_CONST_VEL_GEN_TEST, (int) 5, "", 0, 1, manipulator_name.c_str());

	run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots(1, 1, manipulator_name.c_str(), manipulator_name.c_str());
	//------------------- CONSTANT VELOCITY GENERATOR END -------------------

	//------------------- SMOOTH GENERATOR -------------------
	set_next_ecps_state(ecp_mp::task::ECP_ST_SMOOTH_GEN_TEST, (int) 5, "", 0, 1, manipulator_name.c_str());

	run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots(1, 1, manipulator_name.c_str(), manipulator_name.c_str());
	//------------------- SMOOTH GENERATOR END -------------------

	sr_ecp_msg->message("Gen Test END");

}

} // namespace task
} // namespace mp
} // namespace mrrocpp
