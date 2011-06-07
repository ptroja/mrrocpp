/*
 * g_rotate.cc
 *
 *  Created on: Apr 13, 2010
 *      Author: mmichnie
 */

#include "g_rotate.h"

#include <iostream>

using namespace std;

#define MOTION_STEPS 50
#define EPS 0.02
#define STP 0.02
#define MV 0.0012
#define CORR 0.0//004 korekcja zawodzi-tzn nie jest uzywana symetrycznie ze wzgledu na opor scian!
#define ROT 0.04
#define PI 3.14159

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

const char g_rotate::configSectionName[] = { "[g_rotate]" };

g_rotate::g_rotate(mrrocpp::ecp::common::task::task & _ecp_task) :
	common::generator::generator(_ecp_task)
//:generator(_ecp_task), logEnabled(true)
{
	index = 0;
	sekcja = -1;
	//r = 0.05;
	//k = 0.0;
	//direction = -1;
	//k_max = 0.0;
}

g_rotate::~g_rotate()
{
}
/**
 * direction to move:
 * 0 - -Y up (robot)
 * 1 -  X right
 * 2 -  Y down (computer)
 * 3 - -X left
*/
void g_rotate::configure(double new_rot_position)
{
	index = 0;
	rot_position = new_rot_position;
	sekcja = -1;
	//k = 0.0;
	//direction = new_direction;
	//k_max = new_k_max;
}

bool g_rotate::first_step()
{
	log("g_rotate::first_step()\n");

	the_robot->ecp_command.instruction_type = lib::GET;
	the_robot->ecp_command.get_type = ARM_DEFINITION;
	the_robot->ecp_command.set_type = ARM_DEFINITION;
	the_robot->ecp_command.get_arm_type = lib::FRAME;
	the_robot->ecp_command.set_arm_type = lib::FRAME;

	the_robot->ecp_command.interpolation_type = lib::TCIM;
	the_robot->ecp_command.motion_steps = MOTION_STEPS;
	the_robot->ecp_command.value_in_step_no = MOTION_STEPS - 3;
	the_robot->ecp_command.motion_type = lib::ABSOLUTE;

	log("g_rotate::first_step() end\n");

	return true;
}

bool g_rotate::next_step()
{
	the_robot->ecp_command.instruction_type = lib::SET_GET;
	the_robot->ecp_command.get_type = ARM_DEFINITION;
	the_robot->ecp_command.set_type = ARM_DEFINITION;
	the_robot->ecp_command.get_arm_type = lib::JOINT;
	the_robot->ecp_command.set_arm_type = lib::FRAME;

	the_robot->ecp_command.interpolation_type = lib::TCIM;
	the_robot->ecp_command.motion_steps = MOTION_STEPS;
	the_robot->ecp_command.value_in_step_no = MOTION_STEPS - 3;
	the_robot->ecp_command.motion_type = lib::ABSOLUTE;


	if (index == 0) {
		currentFrame = the_robot->reply_package.arm.pf_def.arm_frame;

		currentFrame.get_translation_vector(first_trans_vect);
		//std::cout << currentFrame << std::endl;


		//mamy pozycje startowa!
		for (size_t i = 0; i < lib::MAX_SERVOS_NR; ++i)
		{
			first_arm_coordinates[i] = the_robot->reply_package.arm.pf_def.arm_coordinates[i];
		}

		GEN_REPLY = 'R';


		index++;
	}

	//lib::Homog_matrix nextFrame;
	//nextFrame = currentFrame;


	double trans_vect[3];
	currentFrame.get_translation_vector(trans_vect);

	lib::Xyz_Euler_Zyz_vector l_vector;
	currentFrame.get_xyz_euler_zyz(l_vector);
	//for(int i=0;i<6;i++)
	//{
	//	std::cout <<"zyz; "<<l_vector[i]<< std::endl;
	//}

	//pobierz actual
	for (size_t i = 0; i < lib::MAX_SERVOS_NR; ++i)
	{
		//std::cout<<the_robot->reply_package.arm.pf_def.arm_coordinates[i]<<endl;

		current_arm_coordinates[i] = the_robot->reply_package.arm.pf_def.arm_coordinates[i];
	}

	//std::cout<<"rot_p: "<<rot_position<<" curr: "<<current_arm_coordinates[5]<<std::endl;

	double theta = 1.57 - current_arm_coordinates[0];

	if(current_arm_coordinates[5] < rot_position + EPS - theta && current_arm_coordinates[5] > rot_position - EPS - theta)
	{
		GEN_REPLY = 'N';
		return false;//bez skretu
	}
	else if(current_arm_coordinates[5] >= rot_position + EPS - theta)
	{
		//current_arm_coordinates[5] -= STP;
		l_vector[5] -= ROT;

		//sekcja do obrotu o 270stopni
		if(current_arm_coordinates[5] < -PI/4)
		{
			sekcja = 3;
		}
		else if(current_arm_coordinates[5] < PI/4)
		{
			sekcja = 2;
		}
		else if(current_arm_coordinates[5] < PI*3/4)
		{
			sekcja = 1;
		}
	}
	else if(current_arm_coordinates[5] <= rot_position - EPS - theta)
	{
		//current_arm_coordinates[5] += STP;
		l_vector[5] += ROT;

		//sekcja do obrotu o 270stopni
		if(current_arm_coordinates[5] > PI/4)
		{
			sekcja = 3;
		}
		else if(current_arm_coordinates[5] > -PI/4)
		{
			sekcja = 2;
		}
		else if(current_arm_coordinates[5] > -PI*3/4)
		{
			sekcja = 1;
		}
	}

	//***PRZESUNIECIE jezeli nie ma duzego oporu
	lib::Ft_v_vector force_torque(the_robot->reply_package.arm.pf_def.force_xyz_torque_xyz);

	double fx = force_torque[0];
	double fy = force_torque[1];
	//double fz = force_torque[2];


	double stop = 7.0;

	std::cout << fx << " " << fy << " " << std::endl;

	if(fx>stop || fx<-stop || fy>stop || fy<-stop)
	{
		GEN_REPLY = 'E';
		return false;
	}


	double wait = 3.0;

	//std::cout << fx << " " << fy << " " << std::endl;

	if(fx>wait || fx<-wait || fy>wait || fy<-wait)
	{
		//return false;
	}
	else
	{
		if(first_arm_coordinates[5] < -PI*3/4 + 2*EPS)//x+
		{
			if(rot_position < 0)//cwierc obrotu
			{
				std::cout <<"z -3/4 w prawo - lewo skreca"<< std::endl;
				l_vector[0] -= MV;
			}
			else //3/4 obrotu
			{
				if(sekcja==1)
				{
					std::cout <<"z -3/4 w (lewo) s1"<< std::endl;
					l_vector[0] -= MV + CORR;
				}
				else if(sekcja==2)
				{
					std::cout <<"z -3/4 w (lewo) s2"<< std::endl;
					l_vector[1] += MV;
				}
				else if(sekcja==3)
				{
					std::cout <<"z -3/4 w (lewo) s3"<< std::endl;
					l_vector[0] += MV - CORR;
				}
			}
		}
		else if(first_arm_coordinates[5] < -PI/4 + 2*EPS)//y-
		{
			if(first_arm_coordinates[5] < rot_position)//lewo
			{
				std::cout <<"z -1/4 w lewo obkrec - ruch w prawo"<< std::endl;
				l_vector[1] += MV;
			}
			else//prawo
			{
				std::cout <<"z -1/4 w prawo obkrec - ruch w lewo"<< std::endl;
				l_vector[0] += MV;
			}

		}
		else if(first_arm_coordinates[5] < PI/4 + 2*EPS)//x-
		{
			if(first_arm_coordinates[5] < rot_position)
			{
				std::cout <<"z 1/4 w lewo obkrec - ruch w prawo"<< std::endl;
				l_vector[0] += MV;
			}
			else
			{
				std::cout <<"z 1/4 w prawo obkrec - ruch w lewo"<< std::endl;
				l_vector[1] -= MV;
			}
		}
		else// if(first_arm_coordinates[5] < PI*3/4 + EPS)//y+
		{
			if(rot_position > 0)//cwierc obrotu w lewo
			{
				std::cout <<"z 3/4 w lewo - prawo skreca"<< std::endl;
				l_vector[0] -= MV;
			}
			else //3/4 obrotu
			{
				if(sekcja==1)
				{
					std::cout <<"z 3/4 w (prawo) s1"<< std::endl;
					l_vector[0] -= MV + CORR;
				}
				else if(sekcja==2)
				{
					std::cout <<"z 3/4 w (prawo) s2"<< std::endl;
					l_vector[1] -= MV;
				}
				else if(sekcja==3)
				{
					std::cout <<"z 3/4 w (prawo) s3"<< std::endl;
					l_vector[0] += MV - CORR;
				}
			}
		}
	}

	currentFrame.set_translation_vector(trans_vect);
	currentFrame.set_from_xyz_euler_zyz(l_vector);
	the_robot->ecp_command.arm.pf_def.arm_frame = currentFrame;

/*
	//pobierz actual
	for (size_t i = 0; i < lib::MAX_SERVOS_NR; ++i)
	{
		current_arm_coordinates[i] = the_robot->reply_package.arm.pf_def.arm_coordinates[i];
	}

	//std::cout<<"rot_p: "<<rot_position<<" curr: "<<current_arm_coordinates[5]<<std::endl;


	double theta = 1.57 - current_arm_coordinates[0];

	if(current_arm_coordinates[5] < rot_position + EPS - theta && current_arm_coordinates[5] > rot_position - EPS - theta)
	{
		return false;//bez skretu
	}
	else if(current_arm_coordinates[5] >= rot_position + EPS - theta)
	{
		current_arm_coordinates[5] -= STP;
	}
	else if(current_arm_coordinates[5] <= rot_position - EPS - theta)
	{
		current_arm_coordinates[5] += STP;
	}

	*///przesuwanie przy obkrecie
	/*
	if(index==1)
	{
		if(first_arm_coordinates[5] < -PI*3/4 + EPS)//x+
		{
			current_arm_coordinates[1] += 0.02;
			current_arm_coordinates[2] += 0.02;
		}
		else if(first_arm_coordinates[5] < -PI/4 + EPS)//y-
		{
			current_arm_coordinates[1] += 0.02;
			current_arm_coordinates[2] -= 0.02;
		}
		else if(first_arm_coordinates[5] < PI/4 + EPS)//x-
		{
			current_arm_coordinates[1] -= 0.02;
			current_arm_coordinates[2] -= 0.02;
		}
		else if(first_arm_coordinates[5] < PI*3/4 + EPS)//y+
		{
			current_arm_coordinates[1] -= 0.02;
			current_arm_coordinates[2] += 0.02;
		}
		index++;
	}
	 */

	//przekaz rozkaz
	/*for (size_t i = 0; i < lib::MAX_SERVOS_NR; ++i)
	{
		the_robot->ecp_command.arm.pf_def.arm_coordinates[i] = current_arm_coordinates[i];
	}
*/



	return true;
} // next_step()

void g_rotate::log(const char *fmt, ...)
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
