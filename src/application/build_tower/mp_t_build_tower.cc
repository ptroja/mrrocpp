/*
 * mp_t_build_tower.cc
 *
 *  Created on: 17-11-2011
 *      Author: spiatek
 */

#include "base/mp/mp_task.h"

#include "base/lib/mrmath/mrmath.h"
#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"
#include "base/lib/sr/srlib.h"

#include "robot/irp6p_m/const_irp6p_m.h"
#include "robot/irp6p_m/mp_r_irp6p_m.h"

#include "generator/ecp/force/ecp_mp_g_tff_gripper_approach.h"

#include "subtask/ecp_mp_st_smooth_file_from_mp.h"
#include "subtask/ecp_mp_st_bias_edp_force.h"

#include "mp_t_build_tower.h"

namespace mrrocpp {
namespace mp {
namespace task {

task* return_created_mp_task(lib::configurator &_config)
{
	return new build_tower(_config);
}

// powolanie robotow w zaleznosci od zawartosci pliku konfiguracyjnego
void build_tower::create_robots()
{
	ACTIVATE_MP_ROBOT(irp6p_m);
}

build_tower::build_tower(lib::configurator &_config) :
	task(_config)
{
}

void build_tower::main_task_algorithm(void)
{
	sr_ecp_msg->message("Build Tower MP Start");

	while(true) {

		sr_ecp_msg->message("Waiting for blue block...");
		set_next_ecp_state(ecp_mp::sub_task::ECP_ST_SMOOTH_JOINT_FILE_FROM_MP, 5, "../../src/application/build_tower/trjs/front_position.trj", 0, lib::irp6p_m::ROBOT_NAME);
		wait_for_task_termination(false, 1, lib::irp6p_m::ROBOT_NAME.c_str());

		//Zerowanie czujników
		sr_ecp_msg->message("Postument Bias");
		set_next_ecp_state(ecp_mp::sub_task::ECP_ST_BIAS_EDP_FORCE, 5, "", 0, lib::irp6p_m::ROBOT_NAME);
		wait_for_task_termination(false, 1, lib::irp6p_m::ROBOT_NAME.c_str());

		wait_ms(8000);

		sr_ecp_msg->message("Reaching the tower place...");		//zakladam, ze pozycja jest niezmienna
		set_next_ecp_state(ecp_mp::sub_task::ECP_ST_SMOOTH_JOINT_FILE_FROM_MP, 5, "../../src/application/build_tower/trjs/pos_build_start.trj", 0, lib::irp6p_m::ROBOT_NAME);
		wait_for_task_termination(false, 1, lib::irp6p_m::ROBOT_NAME.c_str());

		wait_ms(4000);

		sr_ecp_msg->message("Force approach...");
		set_next_ecp_state(ecp_mp::generator::ECP_GEN_TFF_GRIPPER_APPROACH, 5, "", 0, lib::irp6p_m::ROBOT_NAME);
		wait_for_task_termination(false, 1, lib::irp6p_m::ROBOT_NAME.c_str());

		wait_ms(4000);

		//3.1, 1.9 - wymiary klocka

		sr_ecp_msg->message("Raising up...");
		set_next_ecp_state(ecp_mp::sub_task::ECP_ST_SMOOTH_ANGLE_AXIS_FILE_FROM_MP, 5, "../../src/application/build_tower/trjs/build.trj", 0, lib::irp6p_m::ROBOT_NAME);
		wait_for_task_termination(false, 1, lib::irp6p_m::ROBOT_NAME.c_str());

		wait_ms(4000);

	}

	sr_ecp_msg->message("Build Tower END");

}

} // namespace task
} // namespace mp
} // namespace mrrocpp
