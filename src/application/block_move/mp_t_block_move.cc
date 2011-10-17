#include <iostream>
#include <string>
#include <sstream>

#include "base/mp/mp_task.h"
#include "mp_t_block_move.h"

#include "base/lib/mrmath/mrmath.h"

#include "robot/irp6_tfg/dp_tfg.h"
//#include "robot/irp6ot_tfg/const_irp6ot_tfg.h"
#include "robot/irp6p_tfg/const_irp6p_tfg.h"
//#include "robot/irp6ot_m/const_irp6ot_m.h"
#include "robot/irp6p_m/const_irp6p_m.h"

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"
#include "base/lib/sr/srlib.h"

#include "generator/ecp/force/ecp_mp_g_tff_gripper_approach.h"

#include "subtask/ecp_mp_st_smooth_file_from_mp.h"
//#include "subtask/ecp_mp_st_gripper_opening.h"
#include "subtask/ecp_mp_st_bias_edp_force.h"

//#include "robot/irp6ot_m/mp_r_irp6ot_m.h"
#include "robot/irp6p_m/mp_r_irp6p_m.h"
//#include "robot/irp6p_tfg/mp_r_irp6p_tfg.h"

namespace mrrocpp {
namespace mp {
namespace task {

task* return_created_mp_task(lib::configurator &_config)
{
	return new block_move(_config);
}

// powolanie robotow w zaleznosci od zawartosci pliku konfiguracyjnego
void block_move::create_robots()
{
	//ACTIVATE_MP_ROBOT(irp6ot_m);
	ACTIVATE_MP_ROBOT(irp6p_m);
	//ACTIVATE_MP_ROBOT(irp6p_tfg);
}

block_move::block_move(lib::configurator &_config) :
	task(_config)
{
}

void block_move::main_task_algorithm(void)
{
	sr_ecp_msg->message("Block Move MP Start");

	sr_ecp_msg->message("Start position for searching");
	set_next_ecp_state(ecp_mp::sub_task::ECP_ST_SMOOTH_JOINT_FILE_FROM_MP, 5, "../../src/application/block_move/pos_search_area_start.trj", 0, lib::irp6p_m::ROBOT_NAME);
	wait_for_task_termination(false, 1, lib::irp6p_m::ROBOT_NAME.c_str());

	wait_ms(4000);

	sr_ecp_msg->message("Final position for searching");
	set_next_ecp_state(ecp_mp::sub_task::ECP_ST_SMOOTH_JOINT_FILE_FROM_MP, 5, "../../src/application/block_move/pos_search_area_final.trj", 0, lib::irp6p_m::ROBOT_NAME);
	wait_for_task_termination(false, 1, lib::irp6p_m::ROBOT_NAME.c_str());

	//Tutaj powinno być przekazanie parametru i dojście

	//sr_ecp_msg->message("Rozwarcie szczek");
	//set_next_ecp_state(ecp_mp::sub_task::ECP_ST_GRIPPER_OPENING, 0, NULL, 0, lib::irp6p_tfg::ROBOT_NAME);
	//wait_for_task_termination(false, 1, lib::irp6p_tfg::ROBOT_NAME.c_str());

	wait_ms(4000);

	sr_ecp_msg->message("Reach the block");
	set_next_ecp_state(ecp_mp::sub_task::ECP_ST_SMOOTH_ANGLE_AXIS_FILE_FROM_MP, 5, "../../src/application/block_move/block_reaching.trj", 0, lib::irp6p_m::ROBOT_NAME);
	wait_for_task_termination(false, 1, lib::irp6p_m::ROBOT_NAME.c_str());

	//sr_ecp_msg->message("Zwarcie szczek");
	//set_next_ecp_state();
	//wait_for_task_termination(false, 1, lib::irp6p_tfg::ROBOT_NAME.c_str());

	wait_ms(4000);

	sr_ecp_msg->message("Get the block");
	set_next_ecp_state(ecp_mp::sub_task::ECP_ST_SMOOTH_ANGLE_AXIS_FILE_FROM_MP, 5, "../../src/application/block_move/up_to_p0.trj", 0, lib::irp6p_m::ROBOT_NAME);
	wait_for_task_termination(false, 1, lib::irp6p_m::ROBOT_NAME.c_str());

	wait_ms(4000);

	sr_ecp_msg->message("Start position for building");
	set_next_ecp_state(ecp_mp::sub_task::ECP_ST_SMOOTH_JOINT_FILE_FROM_MP, 5, "../../src/application/block_move/pos_build_start.trj", 0, lib::irp6p_m::ROBOT_NAME);
	wait_for_task_termination(false, 1, lib::irp6p_m::ROBOT_NAME.c_str());

	//Tutaj powinno byc przekazanie parametru i dojscie - ruch sklada sie z dwoch czesci

	wait_ms(4000);

	sr_ecp_msg->message("Put the block in its place");
	set_next_ecp_state(ecp_mp::sub_task::ECP_ST_SMOOTH_ANGLE_AXIS_FILE_FROM_MP, 5, "../../src/application/block_move/build.trj", 0, lib::irp6p_m::ROBOT_NAME);
	wait_for_task_termination(false, 1, lib::irp6p_m::ROBOT_NAME.c_str());

	//sr_ecp_msg->message("Rozwarcie szczek");
	//set_next_ecp_state(ecp_mp::sub_task::ECP_ST_GRIPPER_OPENING, 0, NULL, 0, lib::irp6p_tfg::ROBOT_NAME);
	//wait_for_task_termination(false, 1, lib::irp6p_tfg::ROBOT_NAME.c_str());

	//wait_ms(4000);

	//sr_ecp_msg->message("Final position for building");
	//set_next_ecp_state(ecp_mp::sub_task::ECP_ST_SMOOTH_ANGLE_AXIS_FILE_FROM_MP, 5, "../../src/application/block_move/pos_build_final.trj", 0, lib::irp6p_m::ROBOT_NAME);
	//wait_for_task_termination(false, 1, lib::irp6p_m::ROBOT_NAME.c_str());

	//sr_ecp_msg->message("Zwarcie szczek");
	//set_next_ecp_state();
	//wait_for_task_termination(false, 1, lib::irp6p_tfg::ROBOT_NAME.c_str());

	wait_ms(4000);

	sr_ecp_msg->message("Block move END");
}

} // namespace task
} // namespace mp
} // namespace mrrocpp
