/*!
 * @file
 * @brief File contains force_tool_change generator definition
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

#include "base/lib/sr/sr_ecp.h"
#include "base/ecp/ecp_robot.h"
#include "ecp_g_force_tool_change.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // ///////////////////
//
// 			ecp_force_tool_change_generator
//
// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // ///////////////////

force_tool_change::force_tool_change(common::task::task& _ecp_task) :
		common::generator::generator(_ecp_task)
{
	generator_name = ecp_mp::generator::ECP_GEN_FORCE_TOOL_CHANGE;

	set_tool_parameters(-0.18, 0.0, 0.25, 0);
}

bool force_tool_change::first_step()
{

	the_robot->ecp_command.instruction_type = lib::SET;
	the_robot->ecp_command.set_type = ROBOT_MODEL_DEFINITION;
	the_robot->ecp_command.robot_model.type = lib::FORCE_TOOL;

	for (int i = 0; i < 3; i++)
		the_robot->ecp_command.robot_model.force_tool.position[i] = tool_parameters[i];
	the_robot->ecp_command.robot_model.force_tool.weight = weight;

	return true;
}

void force_tool_change::set_tool_parameters(double x, double y, double z, double v)
{
	tool_parameters[0] = x;
	tool_parameters[1] = y;
	tool_parameters[2] = z;
	weight = v;
}

void force_tool_change::conditional_execution()
{
}

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp
