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

g_mm_test::g_mm_test(mrrocpp::ecp::common::task::task & _ecp_task) :
	common::generator::generator(_ecp_task)
//:generator(_ecp_task), logEnabled(true)
{
	index = 0;
	//r = 0.05;
	k = 0.0;
	direction = -1;
	k_max = 0.0;
	licznik_uderzen=0;
	zgubiona_pilka=false;
}

g_mm_test::~g_mm_test()
{
}

void g_mm_test::configure(int new_direction, double new_k_max)
{
	index = 0;
	k = 0.0;
	direction = new_direction;
	k_max = new_k_max;
	licznik_uderzen=0;
	zgubiona_pilka=false;
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


	if (index == 0)
	{
		currentFrame = the_robot->reply_package.arm.pf_def.arm_frame;

		currentFrame.get_translation_vector(first_trans_vect);
		std::cout << currentFrame << std::endl;

		GEN_REPLY = 'M';
	}
	index++;

	log("g_mm_test::next_step() %d\n", index);
	lib::Homog_matrix nextFrame;
	nextFrame = currentFrame;

	//q1 = the_robot->reply_package.arm.pf_def.arm_coordinates[0];
	//double current_arm_coordinates[lib::MAX_SERVOS_NR];
	//for (size_t i = 0; i < lib::MAX_SERVOS_NR; ++i) {
	//	current_arm_coordinates[i] = the_robot->reply_package.arm.pf_def.arm_coordinates[i];
	//		the_robot->ecp_command.arm.pf_def.arm_coordinates[i] = the_robot->reply_package.arm.pf_def.arm_coordinates[i];


	lib::Ft_v_vector force_torque(the_robot->reply_package.arm.pf_def.force_xyz_torque_xyz);

	double fx = force_torque[0];
	double fy = force_torque[1];
	//double fz = force_torque[2];
	std::cout << fx << " " << fy << " " << std::endl;

	/*obecnosc pilki*/
	double granica = 1.2;

	if(abs(fx)>granica || abs(fy)>granica)
	{
		std::cout <<"JEST PILKA"<<std::endl;
		licznik_uderzen+=1;
	}

	if(index > 10)//ustawic, nie za male bo na starcie bedzie sie cofal
	{
		if(licznik_uderzen>1)
		{
			zgubiona_pilka=false;
			std::cout <<"JEST PILKA po 10 index"<<std::endl;
			index=1;
		}
		else
		{
			zgubiona_pilka=true;
			std::cout <<"NIE MA  KULKI"<<std::endl;
			GEN_REPLY = 'E';
			return false;
		}
	}

	/*silne zderzenie ze sciana*/
	double stop = 5.0;

	if(fx>stop || fx<-stop || fy>stop || fy<-stop)
	{
		GEN_REPLY = 'E';
		return false;
	}



	double trans_vect[3];

	/*modyfikuj nextFrame*/
	nextFrame.get_translation_vector(trans_vect);

	//move direction
	if(direction==1)//up
	{
		trans_vect[0] = first_trans_vect[0] - k;
		trans_vect[1] = first_trans_vect[1] + k;
	}
	if(direction==2)//right
	{
		trans_vect[1] = first_trans_vect[1] + k;
		trans_vect[0] = first_trans_vect[0] + k;
	}
	if(direction==3)//down
	{
		trans_vect[0] = first_trans_vect[0] + k;
		trans_vect[1] = first_trans_vect[1] - k;
	}
	if(direction==0)//left
	{
		trans_vect[1] = first_trans_vect[1] - k;
		trans_vect[0] = first_trans_vect[0] - k;
	}

	k += 0.001;
	nextFrame.set_translation_vector(trans_vect);
	/*koniec modyfikacji*/

	the_robot->ecp_command.arm.pf_def.arm_frame = nextFrame;
	currentFrame = nextFrame;

	if (k > k_max)
	{
		GEN_REPLY = 'N';
		return false;
	}

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
	fflush( stdout);
	va_end(ap);
}

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp
