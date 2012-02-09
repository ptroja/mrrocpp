/**
 * @file mp_t_stats.h
 * @brief Header file for stats class.
 * @author Tomasz Bem (mebmot@wp.pl)
 * @ingroup stats
 * @date 25.06.2010
 */

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"

#include "base/mp/mp_task.h"

#include "mp_t_stats.h"
#include "ecp_mp_t_stats.h"

#include "robot/irp6p_m/mp_r_irp6p_m.h"

namespace mrrocpp {
namespace mp {
namespace task {

/*=======================return_created_mp_task===========================*//**
 * @brief returns inherited task pointer.
 * @param _config configurator object reference.
 * @return inherited task pointer.
 */
task* return_created_mp_task(lib::configurator& _config)
{
	return new stats(_config);
}

/*================================constructor=============================*//**
 * @brief Constructor, with task configurator.
 * @param _config configurator object reference.
 */
stats::stats(lib::configurator &_config) :
		task(_config)
{
}

/*================================create_robots===========================*//**
 * @brief Brings robots into being.
 * @details Which robots are bring into being depends on stats.ini
 * configuration file.
 */
void stats::create_robots()
{
	//ACTIVATE_MP_ROBOT(conveyor);
	//
	//
	//ACTIVATE_MP_ROBOT(polycrank);
	//ACTIVATE_MP_ROBOT(bird_hand);
	//ACTIVATE_MP_ROBOT(spkm);
	//ACTIVATE_MP_ROBOT(smb);
	//ACTIVATE_MP_ROBOT(shead);
	//ACTIVATE_MP_ROBOT(irp6ot_tfg);
	//ACTIVATE_MP_ROBOT(irp6ot_m);
	//ACTIVATE_MP_ROBOT(irp6p_tfg);
	ACTIVATE_MP_ROBOT(irp6p_m);
	//ACTIVATE_MP_ROBOT(sarkofag);

	//ACTIVATE_MP_DEFAULT_ROBOT(electron);
	//ACTIVATE_MP_DEFAULT_ROBOT(speechrecognition);
	//ACTIVATE_MP_DEFAULT_ROBOT(festival);

}

/*===============================main_task_algorithm======================*//**
 * @brief Main taks algorithm.
 * @details It initializes stats subtask.
 */
void stats::main_task_algorithm(void)
{
	sr_ecp_msg->message("Stats task initialization");

	set_next_ecp_state(ecp_mp::task::ECP_T_STATS, 5, "", lib::irp6p_m::ROBOT_NAME);
	wait_for_task_termination(false, lib::irp6p_m::ROBOT_NAME);

	sr_ecp_msg->message("END");
}

stats::~stats()
{
}

} //task
} //mp
} //mrrocpp
