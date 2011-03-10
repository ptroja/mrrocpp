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

namespace irp6ot {

namespace generator {

const char g_mm_test::configSectionName[] = { "[g_mm_test]" };

g_mm_test::g_mm_test(mrrocpp::ecp::common::task::task & _ecp_task)
//: mrrocpp::ecp::common::generator::generator(_ecp_task)
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

		the_robot->ecp_command.instruction.instruction_type = lib::GET;
		the_robot->ecp_command.instruction.get_type = ARM_DEFINITION; // arm - ORYGINAL
		the_robot->ecp_command.instruction.set_type = ARM_DEFINITION;
		the_robot->ecp_command.instruction.get_arm_type = lib::FRAME;
		the_robot->ecp_command.instruction.set_arm_type = lib::FRAME;

		the_robot->ecp_command.instruction.interpolation_type = lib::TCIM;
		the_robot->ecp_command.instruction.motion_steps = MOTION_STEPS;
		the_robot->ecp_command.instruction.value_in_step_no = MOTION_STEPS - 3;
		the_robot->ecp_command.instruction.motion_type = lib::ABSOLUTE;

	log("g_mm_test::first_step() end\n");

	return true;
}

bool g_mm_test::next_step()
{
	the_robot->ecp_command.instruction.instruction_type = lib::SET_GET;
	the_robot->ecp_command.instruction.get_type = ARM_DEFINITION; // arm - ORYGINAL
	the_robot->ecp_command.instruction.set_type = ARM_DEFINITION;
	the_robot->ecp_command.instruction.get_arm_type = lib::FRAME;//polozenie w xyz w macierzy 3na4
	the_robot->ecp_command.instruction.set_arm_type = lib::FRAME;

	the_robot->ecp_command.instruction.interpolation_type = lib::TCIM;
	the_robot->ecp_command.instruction.motion_steps = MOTION_STEPS;
	the_robot->ecp_command.instruction.value_in_step_no = MOTION_STEPS - 3;
	the_robot->ecp_command.instruction.motion_type = lib::ABSOLUTE;//polozenie od srodka postumenta

	if(index==0)
	{
		currentFrame.set_from_frame_tab(the_robot->reply_package.arm.pf_def.arm_frame);
		currentGripperCoordinate = the_robot->reply_package.arm.pf_def.gripper_coordinate;
		currentFrame.get_translation_vector(first_trans_vect);//srodek okregu
		//std::cout << currentFrame << std::endl;
		index++;
	}

	log("g_mm_test::next_step() %d\n",index);
	lib::Homog_matrix nextFrame;
	nextFrame = currentFrame;

	double trans_vect [3];

	/*modyfikuj nextFrame*/
	nextFrame.get_translation_vector(trans_vect);

	trans_vect[1]= first_trans_vect[1] + r*sin(k);
	trans_vect[2]= first_trans_vect[2] + r*cos(k) - r;// -r : aby zniwelowac podskok ze srodka okregu na okrag
	k = k+0.1;

	nextFrame.set_translation_vector(trans_vect);
	/*koniec modyfikacji*/

	nextFrame.get_frame_tab(the_robot->ecp_command.instruction.arm.pf_def.arm_frame);
	the_robot->ecp_command.instruction.arm.pf_def.gripper_coordinate = currentGripperCoordinate;
	currentFrame = nextFrame;

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

} // namespace mrrocpp

} // namespace irp6ot

} // namespace ecp

} // namespace mrrocpp
