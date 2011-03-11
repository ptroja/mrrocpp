/*
 * t_mm_test.cc
 *
 *  Created on: Apr 13, 2010
 *      Author: mmichnie
 */
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <iostream>


#include "robot/irp6ot_m/ecp_r_irp6ot_m.h"
#include "robot/irp6p_m/ecp_r_irp6p_m.h"
#include "t_mm_test.h"
//#include "base/lib/datastr.h"


namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

//Constructors
mm_test::mm_test(lib::configurator &_config): common::task::task(_config)
{
	if (config.section_name == lib::irp6ot_m::ECP_SECTION) {
			//ecp_m_robot = (boost::shared_ptr<robot_t>) new irp6ot_m::robot(*this);
			//sg = new common::generator::newsmooth(*this,lib::ECP_JOINT, 7);
		} else if (config.section_name == lib::irp6p_m::ECP_SECTION) {
			ecp_m_robot = (boost::shared_ptr<robot_t>) new irp6p_m::robot(*this);
			sg = new common::generator::newsmooth(*this,lib::ECP_XYZ_ANGLE_AXIS, 6);
		} else {
			// TODO: throw, robot unsupported
			return;
		}

	//my_generator = new generator::g_mm_test(*this);
	sr_ecp_msg->message("ECP loaded mm_test");
};

void mm_test::move_down(double mm)
{
	ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose * actTrajectory = new ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose();
	actTrajectory->arm_type = lib::ECP_XYZ_ANGLE_AXIS;
	for (int i=0;i<6;i++)
	{
		actTrajectory->v.push_back(0.1);
		actTrajectory->a.push_back(0.07);
	}
	actTrajectory->coordinates.push_back(0);
	actTrajectory->coordinates.push_back(0);
	actTrajectory->coordinates.push_back(mm);
	actTrajectory->coordinates.push_back(0);
	actTrajectory->coordinates.push_back(0);
	actTrajectory->coordinates.push_back(0);
	sg->load_relative_pose((*actTrajectory));
}
void mm_test::move_right(double mm)
{
	ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose * actTrajectory = new ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose();
	actTrajectory->arm_type = lib::ECP_XYZ_ANGLE_AXIS;
	for (int i=0;i<6;i++)
	{
		actTrajectory->v.push_back(0.1);
		actTrajectory->a.push_back(0.07);
	}
	actTrajectory->coordinates.push_back(0);
	actTrajectory->coordinates.push_back(mm);
	actTrajectory->coordinates.push_back(0);
	actTrajectory->coordinates.push_back(0);
	actTrajectory->coordinates.push_back(0);
	actTrajectory->coordinates.push_back(0);
	sg->load_relative_pose((*actTrajectory));
}

void mm_test::move_back(double mm)
{
	ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose * actTrajectory = new ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose();
	actTrajectory->arm_type = lib::ECP_XYZ_ANGLE_AXIS;
	for (int i=0;i<6;i++)
	{
		actTrajectory->v.push_back(0.1);
		actTrajectory->a.push_back(0.07);
	}
	actTrajectory->coordinates.push_back(mm);
	actTrajectory->coordinates.push_back(0);
	actTrajectory->coordinates.push_back(0);
	actTrajectory->coordinates.push_back(0);
	actTrajectory->coordinates.push_back(0);
	actTrajectory->coordinates.push_back(0);
	sg->load_relative_pose((*actTrajectory));
}

void mm_test::main_task_algorithm(void ) {

	sr_ecp_msg->message("max's test ready");

	for(;;)
	{
		get_next_state();
		sr_ecp_msg->message("rozkaz odebrany");
		std::string path(mrrocpp_network_path);
		path += (char*)mp_command.ecp_next_state.mp_2_ecp_next_state_string;

		if(((char*)mp_command.ecp_next_state.mp_2_ecp_next_state_string)[0]<= '9' && ((char*)mp_command.ecp_next_state.mp_2_ecp_next_state_string)[0]>= '0')
		{
			double t[2];
			lib::setValuesInArray(t,(char*)mp_command.ecp_next_state.mp_2_ecp_next_state_string);

			if(t[0] < 1.5)
			{
				move_down(t[1]);
			}
			else if(t[0] > 2.5)
			{
				move_back(t[1]);
			}
			else
			{
				move_right(t[1]);
			}
		}
		else
		{
			sg->load_trajectory_from_file(path.c_str());
		}
		sg->calculate_interpolate();
		sg->Move();
		sr_ecp_msg->message("moved");

		ecp_termination_notice();
	sr_ecp_msg->message("noticed");
	}
};

}
} // namespace irp6ot

namespace common {
namespace task {

task_base* return_created_ecp_task(lib::configurator &_config)
{
	return new mm_test(_config);
}

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp


