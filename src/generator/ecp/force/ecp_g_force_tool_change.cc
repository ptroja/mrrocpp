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
#include "generator/ecp/force/ecp_g_force_tool_change.h"

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
	generator(_ecp_task)
{
	set_tool_parameters(-0.18, 0.0, 0.25, 0);
}

bool force_tool_change::first_step()
{
	the_robot->ecp_command.instruction.instruction_type = lib::SET;
	the_robot->ecp_command.instruction.set_type = ROBOT_MODEL_DEFINITION;
	the_robot->ecp_command.instruction.set_robot_model_type = lib::FORCE_TOOL;

	for (int i = 0; i < 3; i++)
		the_robot->ecp_command.instruction.robot_model.force_tool.position[i] = tool_parameters[i];
	the_robot->ecp_command.instruction.robot_model.force_tool.weight = weight;

	return true;
}

bool force_tool_change::next_step()
{
	return false;
}

void force_tool_change::set_tool_parameters(double x, double y, double z, double v)
{
	tool_parameters[0] = x;
	tool_parameters[1] = y;
	tool_parameters[2] = z;
	weight = v;
}

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp
