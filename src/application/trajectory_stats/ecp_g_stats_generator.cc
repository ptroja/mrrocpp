/**
 * @file ecp_g_stats_generator.cc
 * @brief Header file for stats_generator class
 * @author Tomasz Bem (mebmot@wp.pl)
 * @ingroup stats
 * @date 02.07.2010
 */

#include <ctime>

#include "base/ecp/ecp_robot.h"
#include "base/ecp/ecp_task.h"

#include "ecp_g_stats_generator.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

const double MIN_TIME = 3.0;
const double MSTEP_TIME = 0.002 * 10.0;

const double current_ref[] = {15000.0, 18000.0, 10000.0, 10000.0, 10000.0, 10000.0};

static inline void generatePowers(int n, double x, double* powers)
{
	powers[0] = 1.0;
	for (int i = 1; i <= n; i++) {
		powers[i] = powers[i - 1] * x;
	}
}

/*================================Constructor=============================*//**
 * @brief Constructor along with task configurator.
 * @param _ecp_task Reference to task configurator.
 */
stats_generator::stats_generator(common::task::task& _ecp_task) :
	common::generator::generator(_ecp_task)
{
	reset();
}

/*==============================Destructor================================*//**
 * @brief Destructor.
 */
stats_generator::~stats_generator()
{
}

/*===============================first_step===============================*//**
 * @brief First step of stats generator
 * @details Initializes instruction for robot
 */
bool stats_generator::first_step()
{
	sr_ecp_msg.message("stats generator first step");
	printf("stats generator first step\n");
	/*the_robot->ecp_command.instruction_type = lib::GET;
	the_robot->ecp_command.get_type = ARM_DEFINITION;
	the_robot->ecp_command.set_type = ARM_DEFINITION;
	the_robot->ecp_command.set_arm_type = lib::FRAME;
	the_robot->ecp_command.get_arm_type = lib::FRAME;
	the_robot->ecp_command.interpolation_type = lib::MIM;
	the_robot->ecp_command.motion_steps = 10;
	the_robot->ecp_command.value_in_step_no = 10 - 2;
	the_robot->ecp_command.motion_type = lib::ABSOLUTE;*/

	return true;
}
/*================================next_step===============================*//**
 * @brief next step for stats generator.
 */
bool stats_generator::next_step()
{
	printf("next step\n");
	return true;
}

/**
 * @brief Resets generator between consecutive call of Move() method.
 * @details Resets all of the temporary variables. It is necessary to call the
 * reset between the calls of the generator Move() method.
 */
void stats_generator::reset()
{

}

}//generator
}//common
}//ecp
}//mrrocpp
