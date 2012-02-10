/*
 * generator/ecp_g_en_labyrinth.cc
 *
 *Author: Emil Natil
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
#include "ecp_g_en_labyrinth.h"


namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

en_labyrinth::en_labyrinth(common::task::task& _ecp_task) :
		common::generator::newsmooth(_ecp_task, lib::ECP_XYZ_ANGLE_AXIS, 6)
{
	generator_name = ecp_mp::generator::ECP_GEN_EN_LABYRINTH;
}

void en_labyrinth::conditional_execution()
{
	std::string position_string = ecp_t.mp_command.ecp_next_state.sg_buf.get <std::string>();

	std::cout << std::endl << "Reading position from MP: " << position_string << std::endl;
	std::string first_value, buffer;
	std::stringstream position_ss(position_string);

	position_ss >> first_value; // first value in the vector informs about the definition of position values

	std::vector<double> position_vec;
	while (position_ss >> buffer)
	{
		double d;
		std::stringstream ss;
		ss << buffer;
		ss >> d;
		position_vec.push_back(d);
	}

	reset();
	set_absolute();

	if(first_value == "ABSOLUTE_JOIN")
		load_absolute_joint_trajectory_pose(position_vec);
	else if(first_value == "RELATIVE_EULER")
		load_relative_angle_axis_trajectory_pose(position_vec);
	else
		sr_ecp_msg.message("Cannot define coordinates!");

	if(calculate_interpolate())
	{
		Move();
	}
	sr_ecp_msg.message("moved");

}

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp

