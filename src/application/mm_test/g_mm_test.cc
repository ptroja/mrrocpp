/*
 * g_mm_test.cc
 *
 *  Created on: Apr 13, 2010
 *      Author: mmichnie
 */

#include "g_mm_test.h"

#include <iostream>

using namespace std;

#define MOTION_STEPS 50

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

const char g_mm_test::configSectionName[] = { "[g_mm_test]" };

g_mm_test::g_mm_test(mrrocpp::ecp::common::task::task & _ecp_task):
		common::generator::generator(_ecp_task)
//:generator(_ecp_task), logEnabled(true)
{
	index=0;
	r=0.08;
	k = 0.0;
}

g_mm_test::~g_mm_test()
{
}

bool g_mm_test::first_step()
{
	log("g_mm_test::first_step()\n");

		the_robot->ecp_command.instruction_type = lib::GET;
		the_robot->ecp_command.get_type = ARM_DEFINITION; // arm - ORYGINAL
		the_robot->ecp_command.set_type = ARM_DEFINITION;
		the_robot->ecp_command.get_arm_type = lib::FRAME;
		the_robot->ecp_command.set_arm_type = lib::FRAME;

		the_robot->ecp_command.interpolation_type = lib::TCIM;
		the_robot->ecp_command.motion_steps = MOTION_STEPS;
		the_robot->ecp_command.value_in_step_no = MOTION_STEPS - 3;
		the_robot->ecp_command.motion_type = lib::ABSOLUTE;

	log("g_mm_test::first_step() end\n");

	return true;
}

bool g_mm_test::next_step()
{
	the_robot->ecp_command.instruction_type = lib::SET_GET;
	the_robot->ecp_command.get_type = ARM_DEFINITION; // arm - ORYGINAL
	the_robot->ecp_command.set_type = ARM_DEFINITION;
	the_robot->ecp_command.get_arm_type = lib::FRAME;//polozenie w xyz w macierzy 3na4
	the_robot->ecp_command.set_arm_type = lib::FRAME;

	the_robot->ecp_command.interpolation_type = lib::TCIM;
	the_robot->ecp_command.motion_steps = MOTION_STEPS;
	the_robot->ecp_command.value_in_step_no = MOTION_STEPS - 3;
	the_robot->ecp_command.motion_type = lib::ABSOLUTE;//polozenie od srodka postumenta

	if(index==0)
	{
		currentFrame.set_from_frame_tab(the_robot->reply_package.arm.pf_def.arm_frame);

		currentFrame.get_translation_vector(first_trans_vect);//srodek okregu
		//std::cout << currentFrame << std::endl;
		index++;
	}

	log("g_mm_test::next_step() %d\n",index);
	lib::Homog_matrix nextFrame;
	nextFrame = currentFrame;

	lib::Homog_matrix current_frame_wo_offset(the_robot->reply_package.arm.pf_def.arm_frame);
	//current_frame_wo_offset.remove_translation();

	lib::Ft_v_vector force_torque(the_robot->reply_package.arm.pf_def.force_xyz_torque_xyz);
	std::cout << "force: "<<std::endl;
	std::cout<< force_torque << std::endl;



	double trans_vect [3];

	/*modyfikuj nextFrame*/
	nextFrame.get_translation_vector(trans_vect);

	//trans_vect[1]= first_trans_vect[1] + r*sin(k);
	//trans_vect[0]= first_trans_vect[0] + r*cos(k) - r;// -r : aby zniwelowac podskok ze srodka okregu na okrag

	trans_vect[1]= first_trans_vect[1] - r*k;
	//trans_vect[2]= first_trans_vect[2] - r*k;

	k += 0.03;

	nextFrame.set_translation_vector(trans_vect);
	/*koniec modyfikacji*/

	nextFrame.get_frame_tab(the_robot->ecp_command.arm.pf_def.arm_frame);
	currentFrame = nextFrame;

	if(k>1.5)
		return false;

	//std::cout << currentFrame << std::endl;
	//fflush(stdout);


	return true;
} // next_step()

void g_mm_test::log(const char *fmt, ...)
{
	va_list ap;
	va_start(ap, fmt);

	if (!logEnabled) {
		va_end(ap);
		return;
	}

	vfprintf(stdout, fmt, ap);
	fflush(stdout);
	va_end(ap);
}

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp
