/*
 * g_en_labyrinth.cc
 *
 * Author: enatil
 */

#include "g_en_labyrinth.h"

#include <iostream>

using namespace std;

#define MOTION_STEPS 50

using namespace std;

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

const char g_en_lab::configSectionName[] = { "[g_en_lab]" };

g_en_lab::g_en_lab(mrrocpp::ecp::common::task::task & _ecp_task) :
	common::generator::generator(_ecp_task)
//:generator(_ecp_task), logEnabled(true)
{
	index = 0;
	//r = 0.05;
	k = 0.0;
	direction = -1;
	k_max = 0.0;
}

g_en_lab::~g_en_lab()
{
}

void g_en_lab::configure(int new_direction, double new_k_max)
{
	index = 0;
	k = 0.0;
	direction = new_direction;
	k_max = new_k_max;
}

bool g_en_lab::first_step()
{
	log("g_en_lab::first_step()\n");

	the_robot->ecp_command.instruction_type = lib::GET;
	the_robot->ecp_command.get_type = ARM_DEFINITION;
	the_robot->ecp_command.set_type = ARM_DEFINITION;
	the_robot->ecp_command.get_arm_type = lib::FRAME;
	the_robot->ecp_command.set_arm_type = lib::FRAME;

	the_robot->ecp_command.interpolation_type = lib::TCIM;
	the_robot->ecp_command.motion_steps = MOTION_STEPS;
	the_robot->ecp_command.value_in_step_no = MOTION_STEPS - 3;
	the_robot->ecp_command.motion_type = lib::ABSOLUTE;

	log("g_en_lab::first_step() end\n");

	return true;
}

bool g_en_lab::next_step()
{
	the_robot->ecp_command.instruction_type = lib::SET_GET;
	the_robot->ecp_command.get_type = ARM_DEFINITION;
	the_robot->ecp_command.set_type = ARM_DEFINITION;
	the_robot->ecp_command.get_arm_type = lib::FRAME;
	the_robot->ecp_command.set_arm_type = lib::FRAME;

	the_robot->ecp_command.interpolation_type = lib::TCIM;
	the_robot->ecp_command.motion_steps = MOTION_STEPS;
	the_robot->ecp_command.value_in_step_no = MOTION_STEPS - 3;
	the_robot->ecp_command.motion_type = lib::ABSOLUTE;


	if (index == 0)
	{
		currentFrame = the_robot->reply_package.arm.pf_def.arm_frame;

		currentFrame.get_translation_vector(first_trans_vect);
		cout << currentFrame << endl;

		GEN_REPLY = 'M';
	}
	index++;

	log("g_en_lab::next_step() %d\n", index);
	lib::Homog_matrix nextFrame;
	nextFrame = currentFrame;

	lib::Ft_v_vector force_torque(the_robot->reply_package.arm.pf_def.force_xyz_torque_xyz);

	double fx = force_torque[0];
	double fy = force_torque[1];
	double fz = force_torque[2];

//	cout << "Force (x,y,z): (" << fx << "," << fy << "," << fz << ")" << endl;
//	cout.flush();

	double force_limit = 5.0;

	if(fx>force_limit || fx<(-1*force_limit) || fy>force_limit || fy<(-1*force_limit) || fz>force_limit || fz<(-1*force_limit))
	{
		GEN_REPLY = 'E';
		return false;
	}


	double trans_vect[3];

	nextFrame.get_translation_vector(trans_vect);

	if(direction == UP)
	{
		trans_vect[0] = first_trans_vect[0] - k;
		trans_vect[1] = first_trans_vect[1] + k;
	}
	if(direction == RIGHT)
	{
		trans_vect[0] = first_trans_vect[1] + k;
		trans_vect[1] = first_trans_vect[0] + k;
	}
	if(direction == DOWN)
	{
		trans_vect[0] = first_trans_vect[0] + k;
		trans_vect[1] = first_trans_vect[1] - k;
	}
	if(direction == LEFT)
	{
		trans_vect[0] = first_trans_vect[1] - k;
		trans_vect[1] = first_trans_vect[0] - k;
	}

	k += 0.001;
	nextFrame.set_translation_vector(trans_vect);

	the_robot->ecp_command.arm.pf_def.arm_frame = nextFrame;
	currentFrame = nextFrame;

	if (k > k_max)
	{
		GEN_REPLY = 'N';
		return false;
	}

	return true;
} // next_step()

void g_en_lab::log(const char *fmt, ...)
{
	va_list ap;
	va_start(ap, fmt);

	if (!logEnabled) {
		va_end(ap);
		return;
	}

	vfprintf(stdout, fmt, ap);
	fflush( stdout);
	va_end(ap);
}

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp
