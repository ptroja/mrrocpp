
#include <cstdio>
#include <unistd.h>
#include <string.h>
#include <cstring>
#include <iostream>

#include "mp_mm_test.h"
#include "robot/irp6ot_m/mp_r_irp6ot_m.h"
#include "robot/irp6p_m/mp_r_irp6p_m.h"
#include "generator/ecp/ecp_mp_g_newsmooth.h"


#include "base/mp/mp_task.h"
#include "base/mp/MP_main_error.h"
//#include "../edge_follow/mp_t_edge_follow_mr.h"
#include "base/lib/mrmath/mrmath.h"

#include "robot/irp6_tfg/dp_tfg.h"

#include "application/edge_follow/ecp_mp_st_edge_follow.h"
#include "subtask/ecp_mp_st_bias_edp_force.h"
#include "subtask/ecp_mp_st_tff_nose_run.h"
#include "generator/ecp/ecp_mp_g_tfg.h"

#include "robot/irp6ot_tfg/mp_r_irp6ot_tfg.h"
#include "robot/irp6p_tfg/mp_r_irp6p_tfg.h"

#include "ecp_mp_g_g_mm_test.h"
#include "ecp_mp_g_g_rotate.h"

#include "sensor/discode/discode_sensor.h"
#include <boost/shared_ptr.hpp>

#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include "LReading.hpp"

#define PI 3.14159
#define K_MAX 0.0365//0.0365

using mrrocpp::ecp_mp::sensor::discode::discode_sensor;

using namespace std;

namespace mrrocpp {
namespace mp {
namespace task {
task* return_created_mp_task(lib::configurator &_config)
{
	return new mmtest(_config);
}



// powolanie robotow w zaleznosci od zawartosci pliku konfiguracyjnego
void mmtest::create_robots()
{
	//ACTIVATE_MP_ROBOT(irp6ot_m);

	ACTIVATE_MP_ROBOT(irp6p_m);
	ACTIVATE_MP_ROBOT(irp6p_tfg);


}

mmtest::mmtest(lib::configurator &_config) :
		task(_config)
{

	sr_ecp_msg->message("DS test");
	char config_section_name[] = { "[DS Labirynth]" };
	ds = boost::shared_ptr <mrrocpp::ecp_mp::sensor::discode::discode_sensor>(new mrrocpp::ecp_mp::sensor::discode::discode_sensor(config, config_section_name));

	sr_ecp_msg->message("before ds.configure_sensor()\n");
	ds->configure_sensor();
	sr_ecp_msg->message("after ds.configure_sensor()\n");

	ERROR = false;
	path.clear();
}


/*
void mmtest::runWaitFunction(int time)
{
wait_ms(time);
}

void mmtest::runEmptyGen()
{
run_extended_empty_gen_base(state.getNumArgument(), 1, (state.getRobot()).c_str());
}

void mmtest::runEmptyGenForSet(common::State &state)
{
wait_for_task_termination(false,
		state.robotSet->firstSetCount, state.robotSet->secondSetCount, state.robotSet->firstSet,
		state.robotSet->secondSet);
}

void mmtest::executeMotion(common::State &state)
{

int trjConf = config.value<int>("trajectory_from_xml", "[xml_settings]");
if (trjConf && state.getGeneratorType() == ecp_mp::generator::ECP_GEN_NEWSMOOTH) {
	set_next_ecps_state(state.getGeneratorType(), state.getNumArgument(), state.getStateID(), 0, 1,
			(state.getRobot()).c_str());
} else {
	set_next_ecps_state(state.getGeneratorType(), state.getNumArgument(), state.getStringArgument(), 0, 1,
			(state.getRobot()).c_str());
}
}
*/
/*
sr_ecp_msg->message("New edge_follow_mr series");
std::stringstream ss(std::stringstream::in | std::stringstream::out);
lib::Xyz_Euler_Zyz_vector rel_eu(0, 0, 0, -1.57, 1.57, 1.57);
lib::Homog_matrix tmp_hm(rel_eu);
lib::Xyz_Angle_Axis_vector rel_aa;
tmp_hm.get_xyz_angle_axis(rel_aa);
ss << rel_aa;
sr_ecp_msg->message(ss.str().c_str());

set_next_ecps_state(ecp_mp::sub_task::ECP_ST_TFF_NOSE_RUN, (int) 5, "", 0, 1, manipulator_name.c_str());
wait_for_task_termination(false, 1,  manipulator_name.c_str());
set_next_ecps_state(ecp_mp::sub_task::EDGE_FOLLOW, (int) 5, "", 0, 1, manipulator_name.c_str());
wait_for_task_termination(false, 1, manipulator_name.c_str());
*/


void mmtest::set_path()
{
	Point p;
	p.x=3;
	p.y=3;
	path.push_back(p);

//	p.x--;	path.push_back(p);
//	p.y--;	path.push_back(p);
//	p.x--;	path.push_back(p);

	p.x++;	path.push_back(p);
	p.y++;	path.push_back(p);
	p.x--;	path.push_back(p);
	p.y--;	path.push_back(p);

//	p.x--;	path.push_back(p);
//	p.y++;	path.push_back(p);
//	p.y++;	path.push_back(p);
//	p.x++;	path.push_back(p);
//	p.y--;	path.push_back(p);
//	p.x++;	path.push_back(p);
//	p.y++;	path.push_back(p);
//	p.y++;	path.push_back(p);
//	p.x++;	path.push_back(p);
//	p.x++;	path.push_back(p);




	/*
	p.x--;	path.push_back(p);
	p.x--;	path.push_back(p);
	p.x--;	path.push_back(p);

	p.y++;	path.push_back(p);
	p.y++;	path.push_back(p);

	p.x++;	path.push_back(p);

	p.y--;	path.push_back(p);

	p.x++;	path.push_back(p);

	p.y++;	path.push_back(p);
*/
}

void mmtest::main_task_algorithm(void)
{
	sr_ecp_msg->message("mp start");

	/** SET PATH : READ FROM DISCODE */

	double param = 0.0;

	while(true)
	{
		ERROR = false;
		path.clear();

		Types::Mrrocpp_Proxy::LReading lr;

		sr_ecp_msg->message("LR init");
		lr = ds->call_remote_procedure<Types::Mrrocpp_Proxy::LReading>(param);
		sr_ecp_msg->message("LR received");

		while(lr.waiting==true)
		{
			boost::this_thread::sleep(boost::posix_time::milliseconds(500));
			sr_ecp_msg->message("LR restart - waiting");
			lr = ds->call_remote_procedure<Types::Mrrocpp_Proxy::LReading>(double(0.0));
			sr_ecp_msg->message("LR received - waiting");

		}

		std::cout<<"LReading info: "<<lr.path_exists<<std::endl;
		if(!lr.path_exists)
		{
			sr_ecp_msg->message("PROBLEM FROM DISCODE: PATH DOESNOT EXIST");
			return;
		}


		for(int i=49;i>=0;i--)
		{
			if(lr.path[i][0]!=-1 && lr.path[i][1]!=-1)
			{
				Point p;
				p.x=lr.path[i][0];
				p.y=lr.path[i][1];
				path.push_back(p);
			}
		}

		//set_path();


		std::vector<Point>::reverse_iterator xrit,xprit;

		for ( xprit=xrit=path.rbegin(),xrit++ ; xrit < path.rend(); ++xrit, ++xprit )
		{
			std::cout<<"POINT: "<<(*xrit).x<<","<<(*xrit).y<<std::endl;
		}




		/** SET PATH : READ FROM DISCODE **END** */


		lib::robot_name_t manipulator_name;
		lib::robot_name_t gripper_name;

		manipulator_name = lib::irp6p_m::ROBOT_NAME;
		gripper_name = lib::irp6p_tfg::ROBOT_NAME;
		/*

		if(param<1.0)
		{
			sr_ecp_msg->message("SZCZEKI WYSZCZERZ");
			set_next_ecps_state(ecp_mp::generator::ECP_GEN_NEWSMOOTH, (int) 5, "../src/application/mm_test/szczeki2.trj", 0, 1,
				lib::irp6p_m::ROBOT_NAME.c_str());
			wait_for_task_termination(false, 1, lib::irp6p_m::ROBOT_NAME.c_str());


			char tmp_string[lib::MP_2_ECP_NEXT_STATE_STRING_SIZE];
			lib::irp6_tfg::mp_to_ecp_parameters mp_ecp_command;

			mp_ecp_command.desired_position = 0.077;
			memcpy(tmp_string, &mp_ecp_command, sizeof(mp_ecp_command));

			sr_ecp_msg->message("OTWORZ");
			set_next_ecps_state(ecp_mp::generator::ECP_GEN_TFG, (int) 5, tmp_string, sizeof(mp_ecp_command), 1, gripper_name.c_str());
			wait_for_task_termination(false, 1, gripper_name.c_str());

			wait_ms(1000);

			mp_ecp_command.desired_position = 0.066;
			memcpy(tmp_string, &mp_ecp_command, sizeof(mp_ecp_command));

			sr_ecp_msg->message("ZAMKNIJ");
			set_next_ecps_state(ecp_mp::generator::ECP_GEN_TFG, (int) 5, tmp_string, sizeof(mp_ecp_command), 1, gripper_name.c_str());
			wait_for_task_termination(false, 1, gripper_name.c_str());

			wait_ms(2000);
		}

	*/
		/*
		for(int i=0;i<10;i++)
		{
			sr_ecp_msg->message("SZCZEKI W DOL");
			set_next_ecps_state(ecp_mp::generator::ECP_GEN_NEWSMOOTH, (int) 5, "../src/application/mm_test/poz_pocz_bok.trj", 0, 1,
					lib::irp6p_m::ROBOT_NAME.c_str());
			wait_for_task_termination(false, 1, lib::irp6p_m::ROBOT_NAME.c_str());

			char temp_str[20];//args to ecp
			sprintf (temp_str, "%lf %lf", 2.0, K_MAX+i*0.005);//direction, duration in k in arg in frames in time
						set_next_ecps_state(ecp_mp::generator::ECP_GEN_G_MM_TEST, (int) 5, temp_str, 0, 1, lib::irp6p_m::ROBOT_NAME.c_str());
						wait_for_task_termination(false, 1, lib::irp6p_m::ROBOT_NAME.c_str());
		}
	*/


		sr_ecp_msg->message("SZCZEKI W DOL - ladna pozycja nad labiryntem");
		set_next_ecps_state(ecp_mp::generator::ECP_GEN_NEWSMOOTH, (int) 5, "../src/application/mm_test/poz_pocz_bok.trj", 0, 1,
				lib::irp6p_m::ROBOT_NAME.c_str());
		wait_for_task_termination(false, 1, lib::irp6p_m::ROBOT_NAME.c_str());


		/* POZYCJA POCZATKOWA ZANIM SZCZEKI W DOL! */ /* liczona z (3,3) */

		//DODATKOWY OBROT ZEBY NIE BYLO MOZLIWOSCI OBROTU DO TYLU
		char str[20];//args to ecp
		sprintf (str, "%lf", -PI*3/4);//-3/4, -1/4, 1/4, 3/4 - BEZWGLEDNE POLOZENIE!
		set_next_ecps_state(ecp_mp::generator::ECP_GEN_G_ROTATE, (int) 5, str, 0, 1, lib::irp6p_m::ROBOT_NAME.c_str());
		wait_for_task_termination(false,1, lib::irp6p_m::ROBOT_NAME.c_str());

		int sx=3,sy=3;
		int x = lr.path[0][0];//path[0].x;
		int y = lr.path[0][1];//path[0].y;

		std::cout<<"x "<<x<<" y "<<y<<std::endl;
		std::cout<<"sx "<<sx<<" sy "<<sy<<std::endl;

		while(sx!=x || sy!=y)
		{
			char temp[20];//args to ecp

			if(sx<x)
			{
				sprintf (temp, "%lf %lf", 2.0, K_MAX);//direction, duration in k in arg in frames in time
				set_next_ecps_state(ecp_mp::generator::ECP_GEN_G_MM_TEST, (int) 5, temp, 0, 1, lib::irp6p_m::ROBOT_NAME.c_str());
				wait_for_task_termination(false, 1, lib::irp6p_m::ROBOT_NAME.c_str());
				sx++;
				std::cout<<"x "<<x<<" y "<<y<<std::endl;
				std::cout<<"sx "<<sx<<" sy "<<sy<<std::endl;
			}
			if(sx>x)
			{
				sprintf (temp, "%lf %lf", 0.0, K_MAX);//direction, duration in k in arg in frames in time
				set_next_ecps_state(ecp_mp::generator::ECP_GEN_G_MM_TEST, (int) 5, temp, 0, 1, lib::irp6p_m::ROBOT_NAME.c_str());
				wait_for_task_termination(false, 1, lib::irp6p_m::ROBOT_NAME.c_str());
				sx--;
				std::cout<<"x "<<x<<" y "<<y<<std::endl;
				std::cout<<"sx "<<sx<<" sy "<<sy<<std::endl;
			}
			if(sy<y)
			{
				sprintf (temp, "%lf %lf", 3.0, K_MAX);//direction, duration in k in arg in frames in time
				set_next_ecps_state(ecp_mp::generator::ECP_GEN_G_MM_TEST, (int) 5, temp, 0, 1, lib::irp6p_m::ROBOT_NAME.c_str());
				wait_for_task_termination(false, 1, lib::irp6p_m::ROBOT_NAME.c_str());
				sy++;
				std::cout<<"x "<<x<<" y "<<y<<std::endl;
				std::cout<<"sx "<<sx<<" sy "<<sy<<std::endl;
			}
			if(sy>y)
			{
				sprintf (temp, "%lf %lf", 1.0, K_MAX);//direction, duration in k in arg in frames in time
				set_next_ecps_state(ecp_mp::generator::ECP_GEN_G_MM_TEST, (int) 5, temp, 0, 1, lib::irp6p_m::ROBOT_NAME.c_str());
				wait_for_task_termination(false, 1, lib::irp6p_m::ROBOT_NAME.c_str());
				sy--;
				std::cout<<"x "<<x<<" y "<<y<<std::endl;
				std::cout<<"sx "<<sx<<" sy "<<sy<<std::endl;
			}
		}


		/* USTAWIONY */

		sr_ecp_msg->message("SZCZEKI W DOL2");
			set_next_ecps_state(ecp_mp::generator::ECP_GEN_NEWSMOOTH, (int) 5, "../src/application/mm_test/w_dol.trj", 0, 1,
					lib::irp6p_m::ROBOT_NAME.c_str());
			wait_for_task_termination(false, 1, lib::irp6p_m::ROBOT_NAME.c_str());


		//std::cout<<"ECP REPLY: "<<robot_m[manipulator_name/*actual_robot*/]->ecp_reply_package.recognized_command[0]<<std::endl;


		sr_ecp_msg->message("MOJ GENERATOR POPYCHAJACY");

		std::vector<Point>::reverse_iterator rit,prit;

		for ( prit=rit=path.rbegin(),rit++ ; rit < path.rend(); ++rit, ++prit )
		{
			std::cout<<"POINT: "<<(*rit).x<<","<<(*rit).y<<std::endl;

			sr_ecp_msg->message("BIAS");
			set_next_ecps_state(ecp_mp::sub_task::ECP_ST_BIAS_EDP_FORCE, (int) 5, "", 0, 1, manipulator_name.c_str());
			wait_for_task_termination(false, 1,  manipulator_name.c_str());

			char temp_str[20];//args to ecp

			if((*rit).y>(*prit).y)//((*rit).x<(*prit).x)//x-
			{
				sprintf (temp_str, "%lf", PI/4);//-3/4, -1/4, 1/4, 3/4 - BEZWGLEDNE POLOZENIE!
				set_next_ecps_state(ecp_mp::generator::ECP_GEN_G_ROTATE, (int) 5, temp_str, 0, 1, lib::irp6p_m::ROBOT_NAME.c_str());
				wait_for_task_termination(false, 1, lib::irp6p_m::ROBOT_NAME.c_str());

				if (robot_m[manipulator_name/*actual_robot*/]->ecp_reply_package.recognized_command[0] != 'E')
				{
					sprintf (temp_str, "%lf %lf", 3.0, K_MAX);//direction, duration in k in arg in frames in time
					set_next_ecps_state(ecp_mp::generator::ECP_GEN_G_MM_TEST, (int) 5, temp_str, 0, 1, lib::irp6p_m::ROBOT_NAME.c_str());
					wait_for_task_termination(false, 1, lib::irp6p_m::ROBOT_NAME.c_str());
				}
			}
			else if((*rit).y<(*prit).y)//((*rit).x>(*prit).x)//x+
			{
				sprintf (temp_str, "%lf", -PI*3/4);//-3/4, -1/4, 1/4, 3/4 - BEZWGLEDNE POLOZENIE!
				set_next_ecps_state(ecp_mp::generator::ECP_GEN_G_ROTATE, (int) 5, temp_str, 0, 1, lib::irp6p_m::ROBOT_NAME.c_str());
				wait_for_task_termination(false, 1, lib::irp6p_m::ROBOT_NAME.c_str());

				if (robot_m[manipulator_name/*actual_robot*/]->ecp_reply_package.recognized_command[0] != 'E')
				{
					sprintf (temp_str, "%lf %lf", 1.0, K_MAX);//direction, duration in k in arg in frames in time
					set_next_ecps_state(ecp_mp::generator::ECP_GEN_G_MM_TEST, (int) 5, temp_str, 0, 1, lib::irp6p_m::ROBOT_NAME.c_str());
					wait_for_task_termination(false, 1,lib::irp6p_m::ROBOT_NAME.c_str());
				}

			}
			else if((*rit).x>(*prit).x)//((*rit).y<(*prit).y)//y-
			{
				sprintf (temp_str, "%lf", -PI/4);//-3/4, -1/4, 1/4, 3/4 - BEZWGLEDNE POLOZENIE!
				set_next_ecps_state(ecp_mp::generator::ECP_GEN_G_ROTATE, (int) 5, temp_str, 0, 1, lib::irp6p_m::ROBOT_NAME.c_str());
				wait_for_task_termination(false, 1, lib::irp6p_m::ROBOT_NAME.c_str());

				if (robot_m[manipulator_name/*actual_robot*/]->ecp_reply_package.recognized_command[0] != 'E')
				{
					sprintf (temp_str, "%lf %lf", 2.0, K_MAX);//direction, duration in k in arg in frames in time
					set_next_ecps_state(ecp_mp::generator::ECP_GEN_G_MM_TEST, (int) 5, temp_str, 0, 1, lib::irp6p_m::ROBOT_NAME.c_str());
					wait_for_task_termination(false, 1, lib::irp6p_m::ROBOT_NAME.c_str());
				}
			}
			else if((*rit).x<(*prit).x)//((*rit).y>(*prit).y)//y+
			{
				sprintf (temp_str, "%lf", PI*3/4);//-3/4, -1/4, 1/4, 3/4 - BEZWGLEDNE POLOZENIE!
				set_next_ecps_state(ecp_mp::generator::ECP_GEN_G_ROTATE, (int) 5, temp_str, 0, 1, lib::irp6p_m::ROBOT_NAME.c_str());
				wait_for_task_termination(false, 1, lib::irp6p_m::ROBOT_NAME.c_str());

				if (robot_m[manipulator_name/*actual_robot*/]->ecp_reply_package.recognized_command[0] != 'E')
				{
					sprintf (temp_str, "%lf %lf", 0.0, K_MAX);//direction, duration in k in arg in frames in time
					set_next_ecps_state(ecp_mp::generator::ECP_GEN_G_MM_TEST, (int) 5, temp_str, 0, 1, lib::irp6p_m::ROBOT_NAME.c_str());
					wait_for_task_termination(false, 1, lib::irp6p_m::ROBOT_NAME.c_str());
				}
			}

			if (robot_m[manipulator_name/*actual_robot*/]->ecp_reply_package.recognized_command[0] == 'E')
			{
				std::cout<<"EEEEEE BLAD EEEEEEEE"<<std::endl;
				ERROR = true;
				param = param + 1.0;

				//PODNIES SIE
				set_next_ecps_state(ecp_mp::generator::ECP_GEN_NEWSMOOTH, (int) 5, "../src/application/mm_test/w_gore.trj", 0, 1,
						lib::irp6p_m::ROBOT_NAME.c_str());
				wait_for_task_termination(false, 1, lib::irp6p_m::ROBOT_NAME.c_str());
				set_next_ecps_state(ecp_mp::generator::ECP_GEN_NEWSMOOTH, (int) 5, "../src/application/mm_test/w_gore.trj", 0, 1,
						lib::irp6p_m::ROBOT_NAME.c_str());
				wait_for_task_termination(false, 1, lib::irp6p_m::ROBOT_NAME.c_str());

				//WYJEDZ
				sr_ecp_msg->message("WYJEDZ");
				set_next_ecps_state(ecp_mp::generator::ECP_GEN_NEWSMOOTH, (int) 5, "../src/application/mm_test/poz_powtorz.trj", 0, 1,
						lib::irp6p_m::ROBOT_NAME.c_str());
				wait_for_task_termination(false, 1, lib::irp6p_m::ROBOT_NAME.c_str());

				std::cout<<"POWTORZ!"<<std::endl;
				break;
			}

		}
		if(ERROR==false)
			break;//ZAKONCZ
	}//while()

	//PODNIES SIE
	set_next_ecps_state(ecp_mp::generator::ECP_GEN_NEWSMOOTH, (int) 5, "../src/application/mm_test/w_gore.trj", 0, 1,
			lib::irp6p_m::ROBOT_NAME.c_str());
	wait_for_task_termination(false, 1, lib::irp6p_m::ROBOT_NAME.c_str());

//*/

sr_ecp_msg->message("mp end");
}

} // namespace task
} // namespace mp
} // namespace mrrocpp


