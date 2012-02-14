#include "base/mp/mp_task.h"
#include "mp_t_gen_test.h"

#include "base/lib/mrmath/mrmath.h"

#include "robot/irp6_tfg/dp_tfg.h"
#include "robot/irp6ot_tfg/const_irp6ot_tfg.h"
#include "robot/irp6p_tfg/const_irp6p_tfg.h"
#include "robot/irp6ot_m/const_irp6ot_m.h"
#include "robot/irp6p_m/const_irp6p_m.h"
#include "robot/conveyor/const_conveyor.h"

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"
#include <iostream>
#include <string>
#include <sstream>
#include "base/lib/sr/srlib.h"

#include "ecp_mp_t_gen_test.h"

#include "application/generator_tester/ecp_mp_st_smooth_gen_test.h"
#include "application/generator_tester/ecp_mp_st_spline_gen_test.h"
#include "application/generator_tester/ecp_mp_st_const_vel_gen_test.h"

#include "robot/irp6ot_m/mp_r_irp6ot_m.h"
#include "robot/irp6p_m/mp_r_irp6p_m.h"
#include "robot/conveyor/mp_r_conveyor.h"

namespace mrrocpp {
namespace mp {
namespace task {

task* return_created_mp_task(lib::configurator &_config)
{
	return new gen_test(_config);
}

//Robot creation, depending on the configuration file
void gen_test::create_robots()
{
        ACTIVATE_MP_ROBOT(irp6ot_m);
	ACTIVATE_MP_ROBOT(irp6p_m);
        ACTIVATE_MP_ROBOT(conveyor);
}

gen_test::gen_test(lib::configurator &_config) :
		task(_config)
{
}

void gen_test::main_task_algorithm(void)
{

	sr_ecp_msg->message("Gen Test (MP) START");

        //lib::robot_name_t manipulator_name;
        //lib::robot_name_t gripper_name;

	// Track

	/*if (config.value <int> ("is_irp6ot_m_active", lib::UI_SECTION)) {
	 //------------------- CONSTANT VELOCITY GENERATOR -------------------

	 set_next_ecp_state(ecp_mp::subtask::ECP_ST_CONST_VEL_GEN_TEST, (int) 5, "", 0, lib::irp6ot_m::ROBOT_NAME);

	 wait_for_task_termination(false, 1, lib::irp6ot_m::ROBOT_NAME.c_str());

	 //------------------- CONSTANT VELOCITY GENERATOR END -------------------

	 //------------------- SMOOTH GENERATOR -------------------
	 set_next_ecp_state(ecp_mp::generator::ECP_ST_SMOOTH_GEN_TEST, (int) 5, "", 0, lib::irp6ot_m::ROBOT_NAME);

	 wait_for_task_termination(false, 1, lib::irp6ot_m::ROBOT_NAME.c_str());
	 //------------------- SMOOTH GENERATOR END -------------------

	 //------------------- SPLINE GENERATOR -------------------
	 set_next_ecp_state(ecp_mp::subtask::ECP_ST_SPLINE_GEN_TEST, (int) 5, "", 0, lib::irp6ot_m::ROBOT_NAME);

	 wait_for_task_termination(false, 1, lib::irp6ot_m::ROBOT_NAME.c_str());
	 //------------------- SPLINE GENERATOR END -------------------
	 }*/

	// Postument
	if (config.value <int>("is_irp6p_m_active", lib::UI_SECTION)) {
		//------------------- CONSTANT VELOCITY GENERATOR -------------------
		//set_next_ecp_state(ecp_mp::subtask::ECP_ST_CONST_VEL_GEN_TEST, (int) 5, "", lib::irp6p_m::ROBOT_NAME);

		//wait_for_task_termination(false, 1, lib::irp6p_m::ROBOT_NAME.c_str());
		//------------------- CONSTANT VELOCITY GENERATOR END -------------------

		//------------------- SMOOTH GENERATOR -------------------
                set_next_ecp_state(ecp_mp::generator::ECP_MP_SMOOTH_GEN_TEST, (int) 5, "", lib::irp6p_m::ROBOT_NAME);
                wait_for_task_termination(false, lib::irp6p_m::ROBOT_NAME);
		//------------------- SMOOTH GENERATOR END -------------------

		//------------------- SPLINE GENERATOR -------------------
		//set_next_ecp_state(ecp_mp::subtask::ECP_ST_SPLINE_GEN_TEST, (int) 5, "", lib::irp6p_m::ROBOT_NAME);

		//wait_for_task_termination(false, 1, lib::irp6p_m::ROBOT_NAME.c_str());
		//------------------- SPLINE GENERATOR END -------------------
	}

	/*// Conveyor

	 if (config.value <int> ("is_conveyor_active", lib::UI_SECTION)) {
	 //------------------- CONSTANT VELOCITY GENERATOR -------------------
	 set_next_ecp_state(ecp_mp::subtask::ECP_ST_CONST_VEL_GEN_TEST, (int) 5, "", 0, lib::conveyor::ROBOT_NAME);

	 wait_for_task_termination(false, 1, lib::conveyor::ROBOT_NAME.c_str());
	 //------------------- CONSTANT VELOCITY GENERATOR END -------------------

	 //------------------- SMOOTH GENERATOR -------------------
	 set_next_ecp_state(ecp_mp::generator::ECP_ST_SMOOTH_GEN_TEST, (int) 5, "", 0, lib::conveyor::ROBOT_NAME);

	 wait_for_task_termination(false, 1, lib::conveyor::ROBOT_NAME.c_str());
	 //------------------- SMOOTH GENERATOR END -------------------

	 //------------------- SPLINE GENERATOR -------------------
	 set_next_ecp_state(ecp_mp::subtask::ECP_ST_SPLINE_GEN_TEST, (int) 5, "", 0, lib::conveyor::ROBOT_NAME);

	 wait_for_task_termination(false, 1, lib::conveyor::ROBOT_NAME.c_str());
	 //------------------- SPLINE GENERATOR END -------------------
	 }*/

	sr_ecp_msg->message("Gen Test END");

}

} // namespace task
} // namespace mp
} // namespace mrrocpp
