/*!
 * @file
 * @brief File contains force generators definition
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup generators
 */

#include <cstdio>
#include <fstream>
#include <iostream>
#include <ctime>
#include <unistd.h>
#include <cmath>
#include <iostream>

#include "base/lib/typedefs.h"

#include "base/lib/sr/srlib.h"
#include "base/ecp/ecp_robot.h"
#include "generator/ecp/force/ecp_g_bias_edp_force.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // ///////////////////
//
// 			bias_edp_force_generator
//
// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // ///////////////////


bias_edp_force::bias_edp_force(common::task::task& _ecp_task) :
	generator(_ecp_task)
{
}

bool bias_edp_force::first_step()
{

	std::cout << "bias_edp_force" << node_counter << std::endl;

	the_robot->ecp_command.instruction.instruction_type = lib::SET;
	the_robot->ecp_command.instruction.set_type = ROBOT_MODEL_DEFINITION;
	the_robot->ecp_command.instruction.set_robot_model_type = lib::FORCE_BIAS;

	return true;
}

// --------------------------------------------------------------------------


// --------------------------------------------------------------------------
bool bias_edp_force::next_step()
{
	std::cout << "bias_edp_force" << node_counter << std::endl;

	return false;
}

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp
